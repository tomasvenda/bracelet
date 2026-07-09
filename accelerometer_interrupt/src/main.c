#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <math.h>

#include "bmi2_defs.h"
#include "bmi270_legacy.h"

/* Register this module with Zephyr's logging subsystem */
LOG_MODULE_REGISTER(fall_detector, LOG_LEVEL_INF);

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data,
                                    uint32_t len, void *intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data,
                                     uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

#define BMI270_NODE DT_NODELABEL(bmi270)
#define LDO2_NODE   DT_NODELABEL(npm1300_ldsw2)

/* BMI270 hardware registers (direct access, bypasses Bosch API for FIFO) */
#define BMI270_REG_CMD              0x7E
#define BMI270_CMD_FIFO_FLUSH       0xB0
#define BMI270_REG_FIFO_CONFIG_0    0x48
#define BMI270_REG_FIFO_CONFIG_1    0x49
#define BMI270_REG_FIFO_LENGTH_0    0x24
#define BMI270_REG_FIFO_DATA        0x26

/* 1.5s @ 50 Hz = 75 samples × 6 bytes (headerless accel) = 450 bytes each half */
#define HALF_WINDOW_BYTES   450
#define HALF_WINDOW_SAMPLES 75
#define FULL_WINDOW_SAMPLES (HALF_WINDOW_SAMPLES * 2)   /* 150 samples = 3s total */

const struct i2c_dt_spec bmi_i2c = I2C_DT_SPEC_GET(BMI270_NODE);
static const struct gpio_dt_spec bmi_int = GPIO_DT_SPEC_GET(BMI270_NODE, irq_gpios);
static struct gpio_callback bmi_int_cb;
K_SEM_DEFINE(bmi_irq_sem, 0, 1);

static volatile uint32_t isr_count = 0;

/* Raw bytes buffer, reused for each half */
static uint8_t fifo_buffer[HALF_WINDOW_BYTES];

/* Full 3s parsed feature array: past 1.5s + future 1.5s in G-forces */
static float feature_buffer[FULL_WINDOW_SAMPLES * 3];

static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);
    regulator_enable(ldo2_dev);
    k_sleep(K_MSEC(100));
    return 0;
}
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

void bmi_isr_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    isr_count++;
    k_sem_give(&bmi_irq_sem);
}

static void print_rslt(const char *what, int8_t rslt)
{
    if (rslt != BMI2_OK) {
        LOG_ERR("%s: BMI2 error %d", what, rslt);
    } else {
        LOG_INF("%s: OK", what);
    }
}

static int configure_fifo(void)
{
    int ret;
    uint8_t fifo_cfg;
    uint8_t fifo0;

    /* FIFO_CONFIG_0 bit 0 = fifo_stop_on_full. 0 = stream/ring mode */
    i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, &fifo0);
    fifo0 &= ~(1 << 0);
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, fifo0);

    /* FIFO_CONFIG_1: accel-in, gyro-off, headerless */
    ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, &fifo_cfg);
    if (ret) return ret;
    fifo_cfg |= (1 << 6);   /* FIFO_ACC_EN = 1 */
    fifo_cfg &= ~(1 << 5);  /* FIFO_GYR_EN = 0 */
    fifo_cfg &= ~(1 << 4);  /* FIFO_HEADER_EN = 0 */
    ret = i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, fifo_cfg);
    if (ret) return ret;

    /* Flush leftover data */
    return i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
}

/* Read the most recent 1.5s of accel from FIFO, discarding anything older,
 * and write parsed G-force samples into out_features starting at write_offset.
 * Returns number of samples actually written (0..75). */
static uint16_t read_half_window(float *out_features, uint16_t write_offset,
                                 const char *label)
{
    uint8_t len_buf[2];
    int ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_LENGTH_0, len_buf, 2);
    if (ret) {
        LOG_ERR("[%s] FIFO length read failed: %d", label, ret);
        return 0;
    }

    uint16_t fifo_len = len_buf[0] | ((len_buf[1] & 0x1F) << 8);
    uint16_t total_frames = fifo_len / 6;
    uint16_t total_bytes  = total_frames * 6;
    LOG_INF("[%s] FIFO holds %d bytes (%d frames)", label, fifo_len, total_frames);

    if (total_bytes == 0) {
        LOG_WRN("[%s] FIFO empty.", label);
        return 0;
    }

    /* Discard old data if FIFO has more than 1.5s in it */
    if (total_bytes > HALF_WINDOW_BYTES) {
        uint16_t discard = total_bytes - HALF_WINDOW_BYTES;
        uint8_t trash[64];
        while (discard > 0) {
            uint16_t chunk = (discard > sizeof(trash)) ? sizeof(trash) : discard;
            ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, trash, chunk);
            if (ret) {
                LOG_ERR("[%s] FIFO drain failed: %d", label, ret);
                return 0;
            }
            discard -= chunk;
        }
    }

    uint16_t keep_bytes = (total_bytes > HALF_WINDOW_BYTES) ? HALF_WINDOW_BYTES : total_bytes;
    ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, fifo_buffer, keep_bytes);
    if (ret) {
        LOG_ERR("[%s] FIFO data read failed: %d", label, ret);
        return 0;
    }

    uint16_t sample_count = keep_bytes / 6;

    for (uint16_t i = 0; i < sample_count; i++) {
        int16_t raw_x = (int16_t)((fifo_buffer[i*6 + 1] << 8) | fifo_buffer[i*6 + 0]);
        int16_t raw_y = (int16_t)((fifo_buffer[i*6 + 3] << 8) | fifo_buffer[i*6 + 2]);
        int16_t raw_z = (int16_t)((fifo_buffer[i*6 + 5] << 8) | fifo_buffer[i*6 + 4]);
        uint16_t out_idx = (write_offset + i) * 3;
        out_features[out_idx + 0] = raw_x / 16384.0f;
        out_features[out_idx + 1] = raw_y / 16384.0f;
        out_features[out_idx + 2] = raw_z / 16384.0f;
    }

    return sample_count;
}

/* Placeholder for your Edge Impulse inference */
static float run_fall_inference(const float *features, uint16_t n_samples)
{
    uint16_t sub_g_count = 0;
    float min_mag = 999.0f;
    for (uint16_t i = 0; i < n_samples; i++) {
        float x = features[i*3 + 0];
        float y = features[i*3 + 1];
        float z = features[i*3 + 2];
        float mag = sqrtf(x*x + y*y + z*z);
        if (mag < min_mag) min_mag = mag;
        if (mag < 0.4f) sub_g_count++;
    }
    float ratio = (float)sub_g_count / (float)n_samples;
    LOG_INF("Window stats: min_mag=%.3fg  sub_0.4g_ratio=%.2f",
            (double)min_mag, (double)ratio);
    return ratio;
}

int main(void)
{
    int8_t rslt;
    struct bmi2_dev dev = { 0 };
    uint16_t int_status = 0;

    LOG_INF("==========================================================");
    LOG_INF(" BMI270 Fall Detection: past 1.5s + future 1.5s ");
    LOG_INF("==========================================================");

    if (!i2c_is_ready_dt(&bmi_i2c)) {
        LOG_ERR("I2C bus not ready"); return 0;
    }
    if (!gpio_is_ready_dt(&bmi_int)) {
        LOG_ERR("INT GPIO not ready"); return 0;
    }

    dev.intf = BMI2_I2C_INTF;
    dev.read = bmi2_i2c_read;
    dev.write = bmi2_i2c_write;
    dev.delay_us = bmi2_delay_us;
    dev.intf_ptr = NULL;
    dev.read_write_len = 32;

    LOG_INF("[INIT] bmi270_legacy_init...");
    rslt = bmi270_legacy_init(&dev);
    print_rslt("init", rslt);
    if (rslt != BMI2_OK) return 0;

    /* Disable adv_power_save so subsequent feature writes commit reliably */
    rslt = bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
    k_msleep(5);

    /* INT pin electrical config */
    struct bmi2_int_pin_config int_pin_cfg = { 0 };
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od  = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;
    bmi2_set_int_pin_config(&int_pin_cfg, &dev);

    /* Setup Zephyr interrupt (Edge Triggered) */
    gpio_pin_configure_dt(&bmi_int, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&bmi_int_cb, bmi_isr_handler, BIT(bmi_int.pin));
    gpio_add_callback(bmi_int.port, &bmi_int_cb);

    /* ---- ENABLE FIRST (feature engine must own the feature before config) ---- */
    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_LOW_G };
    bmi270_legacy_sensor_enable(sens_list, 2, &dev);

    /* ---- NOW configure accel ---- */
    struct bmi2_sens_config accel_cfg = { .type = BMI2_ACCEL };
    bmi270_legacy_get_sensor_config(&accel_cfg, 1, &dev);
    accel_cfg.cfg.acc.odr         = BMI2_ACC_ODR_50HZ;
    accel_cfg.cfg.acc.range       = BMI2_ACC_RANGE_2G;
    accel_cfg.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
    accel_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    bmi270_legacy_set_sensor_config(&accel_cfg, 1, &dev);

    /* ---- NOW configure low-g feature ---- */
    struct bmi2_sens_config low_g_cfg = { .type = BMI2_LOW_G };
    bmi270_legacy_get_sensor_config(&low_g_cfg, 1, &dev);
    low_g_cfg.cfg.low_g.threshold  = 0x0300;
    low_g_cfg.cfg.low_g.hysteresis = 0x0100;
    low_g_cfg.cfg.low_g.duration   = 0x0002;
    bmi270_legacy_set_sensor_config(&low_g_cfg, 1, &dev);

    /* ---- Map to INT1 last ---- */
    struct bmi2_sens_int_config low_g_int = {
        .type = BMI2_LOW_G, .hw_int_pin = BMI2_INT1
    };
    bmi270_legacy_map_feat_int(&low_g_int, 1, &dev);

    LOG_INF("[INIT] Configuring FIFO...");
    configure_fifo();

    /* Let FIFO fill so first event has full past history */
    k_sleep(K_MSEC(1600));

    LOG_INF(">>> READY. Drop the device or simulate free fall. <<<");

    uint32_t event_num = 0;

    while (1) {
        /* Wait for low-g interrupt */
        k_sem_take(&bmi_irq_sem, K_FOREVER);
        event_num++;
        LOG_INF("=== EVENT #%u (isr_count=%u) ===", event_num, isr_count);

        rslt = bmi2_get_int_status(&int_status, &dev);
        if (rslt != BMI2_OK) {
            LOG_ERR("get_int_status failed: %d", rslt);
            k_sem_reset(&bmi_irq_sem);
            continue;
        }

        if (!(int_status & BMI270_LEGACY_LOW_G_STATUS_MASK)) {
            LOG_WRN("Not a low-g event, ignoring.");
            k_sem_reset(&bmi_irq_sem);
            continue;
        }

        /* --- STEP 1: Capture the PAST 1.5 seconds --- */
        LOG_INF("LOW-G confirmed. Capturing PAST 1.5s from FIFO...");
        uint16_t past_n = read_half_window(feature_buffer, 0, "PAST");

        /* --- STEP 2: Flush FIFO and wait 1.5s to capture the FUTURE --- */
        LOG_INF("Flushing FIFO to record next 1.5s (post-event)...");
        i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
        k_sem_reset(&bmi_irq_sem);  /* ignore any low-g re-triggers during this window */

        k_sleep(K_MSEC(1550));  /* wait a hair longer than 1.5s to be sure FIFO is filled */

        /* --- STEP 3: Capture the FUTURE 1.5 seconds --- */
        LOG_INF("Capturing FUTURE 1.5s from FIFO...");
        uint16_t future_n = read_half_window(feature_buffer, HALF_WINDOW_SAMPLES, "FUTURE");

        uint16_t total_n = past_n + future_n;

        if (total_n > 0) {
            /* Run ML inference over the full 3s window */
            float fall_prob = run_fall_inference(feature_buffer, total_n);
            LOG_INF("Fall probability: %.2f", (double)fall_prob);

            if (fall_prob > 0.5f) {
                LOG_WRN("*** FALL DETECTED — trigger alert here ***");
            } else {
                LOG_INF("Not a fall, dismissing.");
            }
        }

        /* Clear any interrupt latches, flush FIFO so next event starts clean,
         * and let it refill before rearming. */
        bmi2_get_int_status(&int_status, &dev);   /* clears the latched low-g */
        i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
        k_sem_reset(&bmi_irq_sem);
        k_sleep(K_MSEC(1600));   /* refill FIFO for next past-window */

        LOG_INF("Ready for next event.");
    }

    return 0;
}
/* =====================================================================
 * Fall Logger -- ON-DEVICE INFERENCE MODE (Edge Impulse)
 *
 * Pipeline:
 *   boot -> sensor init (same as logger) -> BLUE LED = armed
 *   low-g event -> LED off -> capture 3s window (same PAST/FUTURE
 *   FIFO logic as the logger) -> run Edge Impulse classifier:
 *       "fall"      -> RED LED for 5 s
 *       otherwise   -> GREEN LED for 5 s
 *   -> rearm -> BLUE LED again.
 *
 * DATA CONTRACT (must match the training CSVs):
 *   - 150 frames @ 50 Hz, 4 axes per frame, interleaved:
 *       [x0,y0,z0,dp0, x1,y1,z1,dp1, ...]  (600 floats total)
 *   - accel in g, pressure as DELTA-hPa from the FIRST sample of the
 *     window (baseline subtraction re-enabled below -- same as the
 *     "subtract first row" prep done on the training CSVs).
 *   - the CSV timestamp column was only Studio's time axis; it is NOT
 *     a feature and is not fed to the model.
 * ===================================================================== */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "bmi2_defs.h"
#include "bmi270_legacy.h"
#include "classifier.h"

LOG_MODULE_REGISTER(fall_inference, LOG_LEVEL_INF);

/* =====================================================================
 * HARDWARE CONFIGURATION (identical to the logger)
 * ===================================================================== */
#define BMI270_NODE   DT_NODELABEL(bmi270)
#define ICP20100_NODE DT_NODELABEL(icp20100)
#define LDO2_NODE     DT_NODELABEL(npm1300_ldsw2)

#define BMI270_REG_CMD              0x7E
#define BMI270_CMD_FIFO_FLUSH       0xB0
#define BMI270_REG_FIFO_CONFIG_0    0x48
#define BMI270_REG_FIFO_CONFIG_1    0x49
#define BMI270_REG_FIFO_LENGTH_0    0x24
#define BMI270_REG_FIFO_DATA        0x26

#define ICP20100_REG_MODE_SELECT 0xC0
#define ICP20100_REG_FIFO_FILL   0xC4
#define ICP20100_REG_FIFO_BASE   0xFA
#define ICP20100_REG_DUMMY       0x00
#define ICP20100_FIFO_LEVEL_MASK 0x1F
#define ICP20100_CMD_FIFO_FLUSH  0x80

#define HALF_WINDOW_BYTES   450
#define HALF_WINDOW_SAMPLES 75
#define FULL_WINDOW_SAMPLES (HALF_WINDOW_SAMPLES * 2) /* 150 samples = 3s */
#define MODEL_AXES          4                          /* x, y, z, dp     */

const struct i2c_dt_spec bmi_i2c = I2C_DT_SPEC_GET(BMI270_NODE);
const struct i2c_dt_spec icp_i2c = I2C_DT_SPEC_GET(ICP20100_NODE);
static const struct gpio_dt_spec bmi_int = GPIO_DT_SPEC_GET(BMI270_NODE, irq_gpios);
static struct gpio_callback bmi_int_cb;
K_SEM_DEFINE(bmi_irq_sem, 0, 1);

static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue), gpios);

static uint8_t bmi_fifo_buffer[HALF_WINDOW_BYTES];
static uint8_t icp_fifo_buffer[16 * 6];

struct sensor_record {
    int64_t timestamp;
    double x, y, z;
    double p_hpa;
    bool press_valid;
};

static struct sensor_record event_payload[FULL_WINDOW_SAMPLES];

/* Interleaved feature buffer for the model (static: 2.4 KB) */
static float ei_features[FULL_WINDOW_SAMPLES * MODEL_AXES];

/* =====================================================================
 * LED HELPERS
 *   Blue  = armed & waiting for a drop
 *   (all off = busy capturing/classifying)
 *   Red   = classified as FALL
 *   Green = classified as not-fall
 * ===================================================================== */
static void led_ready_set(bool on)  { gpio_pin_set_dt(&led_blue,  on ? 1 : 0); }
static void led_fall_set(bool on)   { gpio_pin_set_dt(&led_red,   on ? 1 : 0); }
static void led_nofall_set(bool on) { gpio_pin_set_dt(&led_green, on ? 1 : 0); }

/* =====================================================================
 * BOOT SEQUENCE
 * ===================================================================== */
static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);
    if (device_is_ready(ldo2_dev)) {
        regulator_enable(ldo2_dev);
    }
    k_sleep(K_MSEC(100));
    return 0;
}
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

void bmi_isr_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    k_sem_give(&bmi_irq_sem);
}

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

/* =====================================================================
 * SENSOR DRIVERS (identical to the logger)
 * ===================================================================== */
static int configure_bmi_fifo(void)
{
    int ret;
    uint8_t fifo_cfg, fifo0;
    i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, &fifo0);
    fifo0 &= ~(1 << 0);
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, fifo0);

    ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, &fifo_cfg);
    if (ret) return ret;
    fifo_cfg |= (1 << 6);
    fifo_cfg &= ~(1 << 5);
    fifo_cfg &= ~(1 << 4);
    ret = i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, fifo_cfg);
    if (ret) return ret;

    return i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
}

static uint16_t read_bmi_half_window(struct sensor_record *out_buf,
                                     uint16_t write_offset,
                                     const char *label)
{
    uint8_t len_buf[2];
    int ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_LENGTH_0, len_buf, 2);
    if (ret) {
        LOG_ERR("[%s] FIFO length read failed: %d", label, ret);
        return 0;
    }

    uint16_t fifo_len = len_buf[0] | ((len_buf[1] & 0x1F) << 8);
    uint16_t total_bytes  = (fifo_len / 6) * 6;

    if (total_bytes == 0) {
        LOG_WRN("[%s] FIFO empty.", label);
        return 0;
    }

    if (total_bytes > HALF_WINDOW_BYTES) {
        uint16_t discard = total_bytes - HALF_WINDOW_BYTES;
        uint8_t trash[64];
        while (discard > 0) {
            uint16_t chunk = (discard > sizeof(trash)) ? sizeof(trash) : discard;
            ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, trash, chunk);
            if (ret) return 0;
            discard -= chunk;
        }
    }

    uint16_t keep_bytes = (total_bytes > HALF_WINDOW_BYTES) ? HALF_WINDOW_BYTES : total_bytes;
    ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, bmi_fifo_buffer, keep_bytes);
    if (ret) return 0;

    uint16_t sample_count = keep_bytes / 6;

    if ((write_offset + sample_count) > FULL_WINDOW_SAMPLES) {
        sample_count = FULL_WINDOW_SAMPLES - write_offset;
    }

    for (uint16_t i = 0; i < sample_count; i++) {
        int16_t raw_x = (int16_t)((bmi_fifo_buffer[i*6 + 1] << 8) | bmi_fifo_buffer[i*6 + 0]);
        int16_t raw_y = (int16_t)((bmi_fifo_buffer[i*6 + 3] << 8) | bmi_fifo_buffer[i*6 + 2]);
        int16_t raw_z = (int16_t)((bmi_fifo_buffer[i*6 + 5] << 8) | bmi_fifo_buffer[i*6 + 4]);

        out_buf[write_offset + i].x = raw_x / 16384.0;
        out_buf[write_offset + i].y = raw_y / 16384.0;
        out_buf[write_offset + i].z = raw_z / 16384.0;
        out_buf[write_offset + i].timestamp = ((int64_t)(write_offset + i) - HALF_WINDOW_SAMPLES) * 20;
    }

    LOG_INF("[%s] Wrote %d samples (indices %d..%d)",
            label, sample_count, write_offset, write_offset + sample_count - 1);

    return sample_count;
}

static void flush_pressure_fifo(void)
{
    uint8_t current_fill = 0;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &current_fill);
    current_fill |= ICP20100_CMD_FIFO_FLUSH;
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, current_fill);
}

static uint16_t read_pressure_fifo(float *out_pressure, uint16_t max_samples)
{
    uint8_t fifo_fill_reg = 0;
    int ret = i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &fifo_fill_reg);
    if (ret) return 0;

    uint8_t fifo_count = fifo_fill_reg & ICP20100_FIFO_LEVEL_MASK;
    if (fifo_count == 0) return 0;
    if (fifo_count > max_samples) fifo_count = max_samples;

    uint16_t bytes_to_read = fifo_count * 6;
    ret = i2c_burst_read_dt(&icp_i2c, ICP20100_REG_FIFO_BASE, icp_fifo_buffer, bytes_to_read);
    if (ret) return 0;

    uint8_t dummy;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_DUMMY, &dummy);

    for (int i = 0; i < fifo_count; i++) {
        uint8_t *packet = &icp_fifo_buffer[i * 6];
        int32_t data_press = ((int32_t)(packet[2] & 0x0f) << 16) |
                             ((int32_t)packet[1] << 8) |
                             packet[0];

        if (data_press & 0x080000) data_press |= 0xFFF00000;

        out_pressure[i] = ((float)(data_press) * 40.0f / 131072.0f) + 70.0f;
    }
    return fifo_count;
}

/* =====================================================================
 * DATA PROCESSING (identical to logger, EXCEPT baseline subtraction is
 * RE-ENABLED: the model was trained on pressure DELTA from the first
 * row of each CSV, so inference must see the same convention.)
 * ===================================================================== */
static void align_and_pad_pressure(float *past_p, uint16_t past_n, float *future_p, uint16_t future_n)
{
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        event_payload[i].press_valid = false;
    }

    int idx = HALF_WINDOW_SAMPLES - 1 - ((past_n - 1) * 2);
    if (idx < 0) idx = 0;
    for (int i = 0; i < past_n; i++) {
        if (idx >= HALF_WINDOW_SAMPLES) break;
        event_payload[idx].p_hpa = past_p[i] * 10.0f;
        event_payload[idx].press_valid = true;
        idx += 2;
    }

    idx = FULL_WINDOW_SAMPLES - 1 - ((future_n - 1) * 2);
    if (idx < HALF_WINDOW_SAMPLES) idx = HALF_WINDOW_SAMPLES;
    for (int i = 0; i < future_n; i++) {
        if (idx >= FULL_WINDOW_SAMPLES) break;
        event_payload[idx].p_hpa = future_p[i] * 10.0f;
        event_payload[idx].press_valid = true;
        idx += 2;
    }

    double last_p = past_n > 0 ? ((double)past_p[0] * 10.0) : 1013.25;
    bool has_p = false;
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (event_payload[i].press_valid) {
            last_p = event_payload[i].p_hpa;
            has_p = true;
        } else if (has_p) {
            event_payload[i].p_hpa = last_p;
            event_payload[i].press_valid = true;
        }
    }

    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (event_payload[i].press_valid) {
            last_p = event_payload[i].p_hpa;
            break;
        }
    }
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (!event_payload[i].press_valid) {
            event_payload[i].p_hpa = last_p;
            event_payload[i].press_valid = true;
        } else {
            break;
        }
    }

    /* RE-ENABLED: delta pressure, matching the training CSV prep */
    double baseline_p = event_payload[0].p_hpa;
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        event_payload[i].p_hpa = event_payload[i].p_hpa - baseline_p;
    }
}

/* =====================================================================
 * INFERENCE
 * ===================================================================== */
static void classify_event(void)
{
    /* Interleave features exactly as the model was trained:
     * frame-major, axis order acc_x, acc_y, acc_z, pressure_delta */
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        ei_features[i * MODEL_AXES + 0] = (float)event_payload[i].x;
        ei_features[i * MODEL_AXES + 1] = (float)event_payload[i].y;
        ei_features[i * MODEL_AXES + 2] = (float)event_payload[i].z;
        ei_features[i * MODEL_AXES + 3] = (float)event_payload[i].p_hpa;
    }

    char label[32] = {0};
    float score = 0.0f, anomaly = 0.0f;

    int rc = fall_classifier_run(ei_features, ARRAY_SIZE(ei_features),
                                 label, sizeof(label), &score, &anomaly);
    if (rc != 0) {
        LOG_ERR("Classifier failed: %d -- no verdict.", rc);
        return;
    }

    bool is_fall = (strcmp(label, "fall") == 0);

    LOG_INF("=== VERDICT: %s (%.0f%%, anomaly %.2f) -> %s ===",
            label, (double)(score * 100.0f), (double)anomaly,
            is_fall ? "FALL!" : "no fall");

    /* Show the verdict for 5 seconds */
    if (is_fall) {
        led_fall_set(true);
    } else {
        led_nofall_set(true);
    }
    k_sleep(K_MSEC(5000));
    led_fall_set(false);
    led_nofall_set(false);
}

/* =====================================================================
 * LOW-G EVENT HANDLER (capture identical to logger; classify at end)
 * ===================================================================== */
static void handle_low_g_event(struct bmi2_dev *dev)
{
    uint16_t int_status = 0;
    float past_p[16], future_p[16];

    int8_t rslt = bmi2_get_int_status(&int_status, dev);
    if (rslt != BMI2_OK || !(int_status & BMI270_LEGACY_LOW_G_STATUS_MASK)) {
        return;
    }

    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_DISABLE);
    led_ready_set(false);   /* LED off = busy */

    memset(event_payload, 0, sizeof(event_payload));
    int64_t session_id = k_uptime_get();
    LOG_INF("=== LOW-G TRIGGERED (Session: %lld) ===", session_id);

    uint16_t past_acc_n   = read_bmi_half_window(event_payload, 0, "PAST");
    uint16_t past_press_n = read_pressure_fifo(past_p, 16);

    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();

    k_sleep(K_MSEC(1650));

    uint16_t future_acc_n = read_bmi_half_window(event_payload, HALF_WINDOW_SAMPLES, "FUTURE");

    bmi2_set_adv_power_save(BMI2_DISABLE, dev);

    uint16_t future_press_n = read_pressure_fifo(future_p, 16);

    LOG_INF("Captured: past_acc=%d/75 future_acc=%d/75 past_p=%d future_p=%d",
            past_acc_n, future_acc_n, past_press_n, future_press_n);

    if ((past_acc_n + future_acc_n) == FULL_WINDOW_SAMPLES) {
        align_and_pad_pressure(past_p, past_press_n, future_p, future_press_n);
        classify_event();
    } else {
        LOG_WRN("FIFO read short: past=%d future=%d, discarding event.",
                past_acc_n, future_acc_n);
    }

    /* Clean state before rearming */
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();

    LOG_INF("Refilling FIFOs (1.6s)...");
    k_sleep(K_MSEC(1600));

    /* Latched-INT-safe rearm (see logger for full rationale) */
    for (int i = 0; i < 5; i++) {
        bmi2_get_int_status(&int_status, dev);
        if (gpio_pin_get_dt(&bmi_int) == 0) {
            break;
        }
        k_sleep(K_MSEC(10));
    }
    k_sem_reset(&bmi_irq_sem);
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_EDGE_TO_ACTIVE);
    if (gpio_pin_get_dt(&bmi_int) > 0) {
        LOG_WRN("INT already latched at rearm -- forcing handler run.");
        k_sem_give(&bmi_irq_sem);
    }

    led_ready_set(true);
    LOG_INF(">>> Ready for next event. <<<");
}

/* =====================================================================
 * MAIN
 * ===================================================================== */
int main(void)
{
    int8_t rslt;
    struct bmi2_dev dev = { 0 };

    LOG_INF("==========================================================");
    LOG_INF(" Fall Logger: ON-DEVICE INFERENCE MODE (Edge Impulse) ");
    LOG_INF("==========================================================");
    LOG_INF(" Blue = armed | Red = FALL | Green = no fall");

    /* Verify firmware buffer matches the deployed model BEFORE arming */
    LOG_INF("Model expects: %u frames x %u axes = %u features",
            (unsigned)fall_classifier_frame_count(),
            (unsigned)fall_classifier_axes_per_frame(),
            (unsigned)fall_classifier_input_size());
    if (fall_classifier_input_size() != ARRAY_SIZE(ei_features)) {
        LOG_ERR("MODEL/FIRMWARE MISMATCH: firmware provides %u features. "
                "Check window size (3000 ms), frequency (50 Hz) and axis "
                "count in Studio, then redeploy.",
                (unsigned)ARRAY_SIZE(ei_features));
        return 0;
    }

    if (!i2c_is_ready_dt(&bmi_i2c) || !i2c_is_ready_dt(&icp_i2c)) {
        LOG_ERR("I2C bus not ready"); return 0;
    }

    if (gpio_is_ready_dt(&led_red) && gpio_is_ready_dt(&led_green) && gpio_is_ready_dt(&led_blue)) {
        gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&led_blue,  GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_ERR("LED GPIOs not ready");
    }

    LOG_INF("Waiting 5s for power rails to stabilize...");
    k_sleep(K_MSEC(5000));

    /* --- ICP-20100 INIT (with warm-up discard) --- */
    LOG_INF("[INIT] Configuring ICP-20100 (25Hz Continuous)...");
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_MODE_SELECT, 0x08);
    LOG_INF("[INIT] ICP-20100 warm-up (1.2s)...");
    k_sleep(K_MSEC(1200));
    flush_pressure_fifo();

    /* --- BMI270 INIT --- */
    dev.intf = BMI2_I2C_INTF;
    dev.read = bmi2_i2c_read;
    dev.write = bmi2_i2c_write;
    dev.delay_us = bmi2_delay_us;
    dev.intf_ptr = NULL;
    dev.read_write_len = 32;

    rslt = bmi270_legacy_init(&dev);
    if (rslt != BMI2_OK) return 0;

    bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
    k_msleep(5);

    struct bmi2_int_pin_config int_pin_cfg = { 0 };
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od  = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;
    bmi2_set_int_pin_config(&int_pin_cfg, &dev);

    gpio_pin_configure_dt(&bmi_int, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&bmi_int_cb, bmi_isr_handler, BIT(bmi_int.pin));
    gpio_add_callback(bmi_int.port, &bmi_int_cb);

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_LOW_G };
    bmi270_legacy_sensor_enable(sens_list, 2, &dev);

    struct bmi2_sens_config accel_cfg = { .type = BMI2_ACCEL };
    bmi270_legacy_get_sensor_config(&accel_cfg, 1, &dev);
    accel_cfg.cfg.acc.odr         = BMI2_ACC_ODR_50HZ;
    accel_cfg.cfg.acc.range       = BMI2_ACC_RANGE_2G;
    accel_cfg.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
    accel_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    bmi270_legacy_set_sensor_config(&accel_cfg, 1, &dev);

    struct bmi2_sens_config low_g_cfg = { .type = BMI2_LOW_G };
    bmi270_legacy_get_sensor_config(&low_g_cfg, 1, &dev);
    low_g_cfg.cfg.low_g.threshold  = 0x0300;
    low_g_cfg.cfg.low_g.hysteresis = 0x0100;
    low_g_cfg.cfg.low_g.duration   = 0x0002;
    bmi270_legacy_set_sensor_config(&low_g_cfg, 1, &dev);

    struct bmi2_sens_int_config low_g_int = {
        .type = BMI2_LOW_G, .hw_int_pin = BMI2_INT1
    };
    bmi270_legacy_map_feat_int(&low_g_int, 1, &dev);

    LOG_INF("[INIT] Configuring BMI FIFO...");
    configure_bmi_fifo();

    /* Let FIFOs fill before accepting interrupts */
    k_sleep(K_MSEC(1600));

    /* Latched-INT-safe boot arming */
    {
        uint16_t boot_int_status = 0;

        for (int i = 0; i < 5; i++) {
            bmi2_get_int_status(&boot_int_status, &dev);
            if (gpio_pin_get_dt(&bmi_int) == 0) {
                break;
            }
            k_sleep(K_MSEC(10));
        }
        k_sem_reset(&bmi_irq_sem);
        if (gpio_pin_get_dt(&bmi_int) > 0) {
            LOG_WRN("INT latched at boot arm -- forcing handler run.");
            k_sem_give(&bmi_irq_sem);
        }
    }

    led_ready_set(true);   /* Blue LED = armed */
    LOG_INF(">>> READY. Inference active. Waiting for drops. <<<");

    while (1) {
        if (k_sem_take(&bmi_irq_sem, K_MSEC(600)) == 0) {
            handle_low_g_event(&dev);
        } else {
            /* Idle housekeeping: keep the 16-deep pressure FIFO fresh
             * (it stops accepting data when full -- see logger). */
            flush_pressure_fifo();
        }
    }

    return 0;
}
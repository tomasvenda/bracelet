#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/npm13xx_charger.h>

#include "bmi2_defs.h"
#include "bmi270_legacy.h"

#include "events.h"
#include "sensors.h"

LOG_MODULE_REGISTER(sensors_system, LOG_LEVEL_INF);

/* ======================================================================
 * DEVICETREE BINDINGS
 * ====================================================================== */
#define BMI270_NODE    DT_NODELABEL(bmi270)
#define ICP201XX_NODE  DT_NODELABEL(icp20100)
#define BUTTON_NODE    DT_ALIAS(sw0)
#define PWM_CTLR_NODE  DT_NODELABEL(pwm0)
#define LDO1_NODE      DT_NODELABEL(npm1300_ldo1)
#define LDO2_NODE      DT_NODELABEL(npm1300_ldo2)

const struct i2c_dt_spec bmi_i2c = I2C_DT_SPEC_GET(BMI270_NODE);
static const struct i2c_dt_spec icp_i2c = I2C_DT_SPEC_GET(ICP201XX_NODE);
static const struct gpio_dt_spec bmi_int = GPIO_DT_SPEC_GET(BMI270_NODE, irq_gpios);
static const struct gpio_dt_spec button  = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);
static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_CTLR_NODE);

/* Used for battery measurements */
#define NPM1300_CHARGER_NODE DT_NODELABEL(npm1300_charger)
static const struct device *const charger_dev = DEVICE_DT_GET(NPM1300_CHARGER_NODE);

/* ======================================================================
 * BMI270 / ICP-20100 REGISTERS & WINDOW GEOMETRY (from the logger)
 * ====================================================================== */
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
#define ICP20100_REG_FIFO_BASE_PONLY 0xFD   /* pressure-only read start */
#define ICP20100_FIFO_READOUT_PONLY  0x03   /* MODE_SELECT[1:0] = 11 */

#define HALF_WINDOW_BYTES   450
#define HALF_WINDOW_SAMPLES 75
#define FULL_WINDOW_SAMPLES (HALF_WINDOW_SAMPLES * 2)
#define MODEL_AXES          4

/* --- Pressure ring buffer (data-collection builds) ---------------------
 * Mode-0 FIFO is stop-on-full and fills in ~640 ms, so the sensor thread
 * drains it into this RAM ring to keep a rolling pre-impact history. */
#define BARO_ODR_HZ        25.0f          /* ICP mode 0 */
#define BARO_RING_SAMPLES  128            /* ~5 s @ 25 Hz */

/* ======================================================================
 * MODULE STATE
 * ====================================================================== */
static struct bmi2_dev bmi_dev_ctx;
static struct gpio_callback bmi_int_cb;
static struct gpio_callback button_cb_data;
static K_SEM_DEFINE(bmi_irq_sem, 0, 1);

/* Set while a fall-window capture is running: suppresses the idle
* pressure flush and any further harsh-impact events. */
static atomic_t sensors_ready = ATOMIC_INIT(0);
static atomic_t capture_pending = ATOMIC_INIT(0);

static uint8_t bmi_fifo_buffer[HALF_WINDOW_BYTES];
static uint8_t icp_fifo_buffer[16 * 3];

struct sensor_record {
    double x, y, z;
    double p_hpa;
    bool press_valid;
};
static struct sensor_record event_payload[TRAIN_TOTAL_SAMPLES];

/* Rolling pressure history in hPa, written by the sensor thread. */
static float          baro_ring[BARO_RING_SAMPLES];
static uint16_t       baro_ring_head;    /* next write slot        */
static uint16_t       baro_ring_count;   /* valid entries (<= size) */
static K_MUTEX_DEFINE(baro_ring_lock);

/* Bosch API glue -- implemented in bmi270_legacy_api (same as before) */
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);
static void rearm_bmi_interrupt(void);

/* ======================================================================
 * PWM: LED & BUZZER (unchanged from previous version)
 * ====================================================================== */
#define PWM_CH_RED     0U
#define PWM_CH_GREEN   1U
#define PWM_CH_BLUE    2U
#define PWM_CH_BUZZER  3U
#define PWM_UNIFIED_PERIOD  PWM_HZ(4000)
#define PWM_LED_PULSE       (PWM_UNIFIED_PERIOD / 2U)
#define PWM_BUZZ_PULSE      (PWM_UNIFIED_PERIOD / 2U)

void sensors_led_on(uint8_t color)
{
    if (color & LED_RED)   pwm_set(pwm_dev, PWM_CH_RED,   PWM_UNIFIED_PERIOD, PWM_LED_PULSE, PWM_POLARITY_NORMAL);
    if (color & LED_GREEN) pwm_set(pwm_dev, PWM_CH_GREEN, PWM_UNIFIED_PERIOD, PWM_LED_PULSE, PWM_POLARITY_NORMAL);
    if (color & LED_BLUE)  pwm_set(pwm_dev, PWM_CH_BLUE,  PWM_UNIFIED_PERIOD, PWM_LED_PULSE, PWM_POLARITY_NORMAL);
}

void sensors_led_off(uint8_t color)
{
    if (color & LED_RED)   pwm_set(pwm_dev, PWM_CH_RED,   PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);
    if (color & LED_GREEN) pwm_set(pwm_dev, PWM_CH_GREEN, PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);
    if (color & LED_BLUE)  pwm_set(pwm_dev, PWM_CH_BLUE,  PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);
}

void sensors_buzzer_on(void)
{
    pwm_set(pwm_dev, PWM_CH_BUZZER, PWM_UNIFIED_PERIOD, PWM_BUZZ_PULSE, PWM_POLARITY_NORMAL);
}

void sensors_buzzer_off(void)
{
    pwm_set(pwm_dev, PWM_CH_BUZZER, PWM_UNIFIED_PERIOD, 0U, PWM_POLARITY_NORMAL);
}

void sensors_evaluation_done(void)
{
    atomic_clear(&capture_pending);
    rearm_bmi_interrupt();
}

/* ======================================================================
 * ISRs -- fast paths only (no I2C allowed here)
 * ====================================================================== */
static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    static uint32_t last_press_time = 0;
    uint32_t now = k_uptime_get_32();
    if (now - last_press_time < 500) return;
    last_press_time = now;

    struct bracelet_event event = { .type = EVENT_BUTTON_PRESSED };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

static void bmi_isr(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    /* Which feature fired is in an I2C register -- defer to the
     * sensor thread. */
    k_sem_give(&bmi_irq_sem);
}

/* ======================================================================
 * PRESSURE HELPERS (from the logger)
 * ====================================================================== */
static void flush_pressure_fifo(void)
{
    uint8_t fill = 0;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &fill);
    fill |= ICP20100_CMD_FIFO_FLUSH;
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, fill);
}

static uint16_t read_pressure_fifo(float *out_pressure, uint16_t max_samples)
{
    uint8_t fill_reg = 0;
    if (i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &fill_reg)) return 0;

    uint8_t count = fill_reg & ICP20100_FIFO_LEVEL_MASK;
    if (count == 0) return 0;
    if (count > max_samples) count = max_samples;

    if (i2c_burst_read_dt(&icp_i2c, ICP20100_REG_FIFO_BASE_PONLY, icp_fifo_buffer, count * 3)) return 0;

    uint8_t dummy;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_DUMMY, &dummy);

    for (int i = 0; i < count; i++) {
        uint8_t *pkt = &icp_fifo_buffer[i * 3];
        int32_t raw = ((int32_t)(pkt[2] & 0x0f) << 16) | ((int32_t)pkt[1] << 8) | pkt[0];
        if (raw & 0x080000) raw |= 0xFFF00000;
        out_pressure[i] = ((float)raw * 40.0f / 131072.0f) + 70.0f;
    }
    return count;
}

/* --- Pressure ring buffer helpers (store hPa) -------------------------- */
static void baro_ring_push(float p_hpa)
{
    baro_ring[baro_ring_head] = p_hpa;
    baro_ring_head = (baro_ring_head + 1) % BARO_RING_SAMPLES;
    if (baro_ring_count < BARO_RING_SAMPLES) baro_ring_count++;
}

/* Pull whatever is staged in the HW FIFO into the ring (oldest-first). */
static void baro_drain_to_ring(void)
{
    float tmp[16];
    uint16_t n = read_pressure_fifo(tmp, 16);          /* kPa */
    if (n == 0) return;
    k_mutex_lock(&baro_ring_lock, K_FOREVER);
    for (uint16_t i = 0; i < n; i++) baro_ring_push(tmp[i] * 10.0f);  /* -> hPa */
    k_mutex_unlock(&baro_ring_lock);
}

/* Copy the newest `want` ring samples into out[] oldest-first;
 * returns how many were available (<= want). */
static uint16_t baro_ring_snapshot(float *out, uint16_t want)
{
    k_mutex_lock(&baro_ring_lock, K_FOREVER);
    uint16_t n = (baro_ring_count < want) ? baro_ring_count : want;
    uint16_t idx = (baro_ring_head + BARO_RING_SAMPLES - n) % BARO_RING_SAMPLES;
    for (uint16_t i = 0; i < n; i++) {
        out[i] = baro_ring[idx];
        idx = (idx + 1) % BARO_RING_SAMPLES;
    }
    k_mutex_unlock(&baro_ring_lock);
    return n;
}

static void baro_ring_reset(void)
{
    k_mutex_lock(&baro_ring_lock, K_FOREVER);
    baro_ring_head = 0; baro_ring_count = 0;
    k_mutex_unlock(&baro_ring_lock);
}

uint16_t sensors_debug_read_pressure(float *out_hpa, uint16_t max_samples, uint32_t wait_ms)
{
    atomic_set(&capture_pending, 1);      /* suppress idle flush, like a real capture */
    flush_pressure_fifo();
    sensors_led_on(LED_BLUE);             /* LED ON = accumulating */
    k_sleep(K_MSEC(wait_ms - 700));
    sensors_led_off(LED_BLUE);            /* LED OFF = raise NOW */
    k_sleep(K_MSEC(700));
    uint16_t n = read_pressure_fifo(out_hpa, max_samples);
    atomic_set(&capture_pending, 0);
    return n;
}

/* ======================================================================
 * BMI270 FIFO HELPERS (from the logger)
 * ====================================================================== */
static int configure_bmi_fifo(void)
{
    int ret;
    uint8_t fifo_cfg, fifo0;
    i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, &fifo0);
    fifo0 &= ~(1 << 0);
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, fifo0);

    ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, &fifo_cfg);
    if (ret) return ret;
    fifo_cfg |= (1 << 6);   /* ACC_EN */
    fifo_cfg &= ~(1 << 5);  /* GYR off */
    fifo_cfg &= ~(1 << 4);  /* headerless */
    ret = i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, fifo_cfg);
    if (ret) return ret;

    return i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
}

static uint16_t read_bmi_half_window(struct sensor_record *out_buf,
                                     uint16_t write_offset, const char *label)
{
    uint8_t len_buf[2];
    if (i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_LENGTH_0, len_buf, 2)) {
        LOG_ERR("[%s] FIFO length read failed", label);
        return 0;
    }

    uint16_t fifo_len = len_buf[0] | ((len_buf[1] & 0x1F) << 8);
    uint16_t total_bytes = (fifo_len / 6) * 6;
    if (total_bytes == 0) {
        LOG_WRN("[%s] FIFO empty.", label);
        return 0;
    }

    if (total_bytes > HALF_WINDOW_BYTES) {
        uint16_t discard = total_bytes - HALF_WINDOW_BYTES;
        uint8_t trash[64];
        while (discard > 0) {
            uint16_t chunk = (discard > sizeof(trash)) ? sizeof(trash) : discard;
            if (i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, trash, chunk)) return 0;
            discard -= chunk;
        }
    }

    uint16_t keep = (total_bytes > HALF_WINDOW_BYTES) ? HALF_WINDOW_BYTES : total_bytes;
    if (i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, bmi_fifo_buffer, keep)) return 0;

    uint16_t n = keep / 6;
    if ((write_offset + n) > FULL_WINDOW_SAMPLES) {
        n = FULL_WINDOW_SAMPLES - write_offset;
    }

    for (uint16_t i = 0; i < n; i++) {
        int16_t rx = (int16_t)((bmi_fifo_buffer[i*6+1] << 8) | bmi_fifo_buffer[i*6+0]);
        int16_t ry = (int16_t)((bmi_fifo_buffer[i*6+3] << 8) | bmi_fifo_buffer[i*6+2]);
        int16_t rz = (int16_t)((bmi_fifo_buffer[i*6+5] << 8) | bmi_fifo_buffer[i*6+4]);
        out_buf[write_offset+i].x = rx / 16384.0;
        out_buf[write_offset+i].y = ry / 16384.0;
        out_buf[write_offset+i].z = rz / 16384.0;
    }

    LOG_INF("[%s] Wrote %d samples (indices %d..%d)", label, n,
            write_offset, write_offset + n - 1);
    return n;
}

/* Read up to want_frames newest accel frames (drops oldest overflow). */
static uint16_t read_bmi_frames(struct sensor_record *out, uint16_t off,
                                uint16_t want_frames, const char *label)
{
    uint8_t len_buf[2];
    if (i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_LENGTH_0, len_buf, 2)) return 0;
    uint16_t total = ((len_buf[0] | ((len_buf[1] & 0x1F) << 8)) / 6) * 6;
    uint16_t want  = want_frames * 6;

    if (total > want) {                       /* discard oldest */
        uint16_t discard = total - want;
        uint8_t trash[64];
        while (discard) {
            uint16_t c = MIN(discard, sizeof(trash));
            if (i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, trash, c)) return 0;
            discard -= c;
        }
        total = want;
    }

    uint16_t done = 0;
    while (done < total) {                    /* parse in 450 B chunks */
        uint16_t c = MIN(total - done, (uint16_t)sizeof(bmi_fifo_buffer));
        if (i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, bmi_fifo_buffer, c)) break;
        for (uint16_t i = 0; i < c / 6; i++) {
            uint16_t k = off + done / 6 + i;
            if (k >= TRAIN_TOTAL_SAMPLES) break;
            out[k].x = (int16_t)((bmi_fifo_buffer[i*6+1] << 8) | bmi_fifo_buffer[i*6+0]) / 16384.0;
            out[k].y = (int16_t)((bmi_fifo_buffer[i*6+3] << 8) | bmi_fifo_buffer[i*6+2]) / 16384.0;
            out[k].z = (int16_t)((bmi_fifo_buffer[i*6+5] << 8) | bmi_fifo_buffer[i*6+4]) / 16384.0;
        }
        done += c;
    }
    LOG_INF("[%s] %u frames", label, done / 6);
    return done / 6;
}



/* ======================================================================
 * PRESSURE ALIGNMENT (from the logger; baseline subtraction ENABLED --
 * the model was trained on delta-from-first-sample)
 * ====================================================================== */
static void align_and_pad_pressure(float *past_p, uint16_t past_n,
                                   float *future_p, uint16_t future_n)
{
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) event_payload[i].press_valid = false;

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
            last_p = event_payload[i].p_hpa; has_p = true;
        } else if (has_p) {
            event_payload[i].p_hpa = last_p;
            event_payload[i].press_valid = true;
        }
    }
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (event_payload[i].press_valid) { last_p = event_payload[i].p_hpa; break; }
    }
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (!event_payload[i].press_valid) {
            event_payload[i].p_hpa = last_p;
            event_payload[i].press_valid = true;
        } else break;
    }

    double baseline = event_payload[0].p_hpa;
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        event_payload[i].p_hpa -= baseline;
    }
}

/* Linear-interpolate pressure (m samples @ f_baro Hz) onto an n-sample
 * accel window @ 50 Hz. Both are assumed to END at the same instant (the
 * newest sample sits at the impact edge). Ends clamp. ABSOLUTE hPa. */
static void interpolate_pressure_to_accel(const float *press, int m, float f_baro,
                                          struct sensor_record *rec, int n)
{
    if (m <= 0) {
        for (int i = 0; i < n; i++) { rec[i].p_hpa = 0.0; rec[i].press_valid = false; }
        return;
    }
    if (m == 1) {
        for (int i = 0; i < n; i++) { rec[i].p_hpa = press[0]; rec[i].press_valid = true; }
        return;
    }
    const double dt_acc   = 1.0 / 50.0;
    const double dt_press = 1.0 / (double)f_baro;
    for (int i = 0; i < n; i++) {
        double t  = (double)(i - (n - 1)) * dt_acc;        /* <= 0, newest at 0 */
        double fj = (double)(m - 1) + t / dt_press;
        if      (fj <= 0.0)            rec[i].p_hpa = press[0];
        else if (fj >= (double)(m-1))  rec[i].p_hpa = press[m - 1];
        else {
            int j = (int)fj;
            double frac = fj - (double)j;
            rec[i].p_hpa = press[j] + frac * (press[j + 1] - press[j]);
        }
        rec[i].press_valid = true;
    }
}

/* ======================================================================
 * LATCHED-INT-SAFE REARM (from the logger)
 * ====================================================================== */
static void rearm_bmi_interrupt(void)
{
    uint16_t int_status = 0;

    for (int i = 0; i < 5; i++) {
        bmi2_get_int_status(&int_status, &bmi_dev_ctx);
        if (gpio_pin_get_dt(&bmi_int) == 0) break;
        k_sleep(K_MSEC(10));
    }
    k_sem_reset(&bmi_irq_sem);
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_EDGE_TO_ACTIVE);
    if (gpio_pin_get_dt(&bmi_int) > 0) {
        LOG_WRN("INT still latched at rearm -- forcing handler run.");
        k_sem_give(&bmi_irq_sem);
    }
}

/* ======================================================================
 * PUBLIC: FALL-WINDOW CAPTURE (called from the FSM's evaluation k_work)
 * ====================================================================== */
int sensors_capture_fall_window(float *features, size_t count)
{
    float past_p[16], future_p[16];

    if (count != FULL_WINDOW_SAMPLES * MODEL_AXES) {
        LOG_ERR("Capture buffer is %u floats, need %u",
                (unsigned)count, FULL_WINDOW_SAMPLES * MODEL_AXES);
        return -EINVAL;
    }

    /* Mask INT1 while we're busy for several seconds */
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_DISABLE);

    memset(event_payload, 0, sizeof(event_payload));
    LOG_INF("=== FALL WINDOW CAPTURE (t=%lld) ===", k_uptime_get());

    uint16_t past_acc_n   = read_bmi_half_window(event_payload, 0, "PAST");
    uint16_t past_press_n = read_pressure_fifo(past_p, 16);

    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();

    k_sleep(K_MSEC(1650));

    uint16_t future_acc_n = read_bmi_half_window(event_payload, HALF_WINDOW_SAMPLES, "FUTURE");
    bmi2_set_adv_power_save(BMI2_DISABLE, &bmi_dev_ctx);
    uint16_t future_press_n = read_pressure_fifo(future_p, 16);

    LOG_INF("Captured: past_acc=%d/75 future_acc=%d/75 past_p=%d future_p=%d",
            past_acc_n, future_acc_n, past_press_n, future_press_n);

    int ret = 0;
    if ((past_acc_n + future_acc_n) == FULL_WINDOW_SAMPLES) {
        align_and_pad_pressure(past_p, past_press_n, future_p, future_press_n);
        for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
            features[i * MODEL_AXES + 0] = (float)event_payload[i].x;
            features[i * MODEL_AXES + 1] = (float)event_payload[i].y;
            features[i * MODEL_AXES + 2] = (float)event_payload[i].z;
            features[i * MODEL_AXES + 3] = (float)event_payload[i].p_hpa;
        }
    } else {
        LOG_WRN("FIFO read short -- discarding event.");
        ret = -EIO;
    }

    /* Clean state, refill, rearm */
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();
    LOG_INF("Refilling FIFOs (1.6s)...");
    k_sleep(K_MSEC(1600));
    rearm_bmi_interrupt();

    return ret;
}

int sensors_capture_training_window(float *features, size_t count)
{
    float past_p[64], future_p[96];

    if (count != TRAIN_TOTAL_SAMPLES * MODEL_AXES) return -EINVAL;

    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_DISABLE);
    memset(event_payload, 0, sizeof(event_payload));
    LOG_INF("=== TRAINING WINDOW CAPTURE (t=%lld) ===", k_uptime_get());

    /* PAST accel from the BMI FIFO. PAST pressure from the ring (the thread
     * has been draining ~5 s of history into it). Drain once more first so
     * the ring's newest sample lands right at the impact edge. */
    uint16_t past_acc_n = read_bmi_frames(event_payload, 0, TRAIN_PAST_SAMPLES, "PAST");
    baro_drain_to_ring();
    uint16_t past_press_n = baro_ring_snapshot(past_p, 40);   /* ~1.6 s @ 25 Hz */

    /* Flush both, then collect the 3 s FUTURE pressure in drained chunks so
     * the 16-deep FIFO never overflows across the window. */
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();

    uint16_t future_press_n = 0;
    for (int c = 0; c < 6; c++) {                 /* 6 x 510 ms = 3.06 s */
        k_sleep(K_MSEC(510));
        uint16_t got = read_pressure_fifo(&future_p[future_press_n],
                                          ARRAY_SIZE(future_p) - future_press_n);
        for (uint16_t i = 0; i < got; i++)        /* kPa -> hPa */
            future_p[future_press_n + i] *= 10.0f;
        future_press_n += got;
    }
    uint16_t future_acc_n = read_bmi_frames(event_payload, TRAIN_PAST_SAMPLES,
                                            TRAIN_FUTURE_SAMPLES, "FUTURE");
    bmi2_set_adv_power_save(BMI2_DISABLE, &bmi_dev_ctx);

    LOG_INF("acc %u/%u + %u/%u, press %u + %u",
            past_acc_n, TRAIN_PAST_SAMPLES, future_acc_n, TRAIN_FUTURE_SAMPLES,
            past_press_n, future_press_n);

    int ret = 0;
    if (past_acc_n == TRAIN_PAST_SAMPLES && future_acc_n == TRAIN_FUTURE_SAMPLES) {
        /* PAST: newest pressure anchored to impact (last past index).
         * FUTURE: oldest pressure anchored at impact (first future index). */
        interpolate_pressure_to_accel(past_p, past_press_n, BARO_ODR_HZ,
                                      &event_payload[0], TRAIN_PAST_SAMPLES);
        interpolate_pressure_to_accel(future_p, future_press_n, BARO_ODR_HZ,
                                      &event_payload[TRAIN_PAST_SAMPLES], TRAIN_FUTURE_SAMPLES);

        for (int i = 0; i < TRAIN_TOTAL_SAMPLES; i++) {
            features[i*4+0] = (float)event_payload[i].x;
            features[i*4+1] = (float)event_payload[i].y;
            features[i*4+2] = (float)event_payload[i].z;
            features[i*4+3] = (float)event_payload[i].p_hpa;   /* ABSOLUTE hPa */
        }
    } else {
        LOG_WRN("Short FIFO read -- discarding event.");
        ret = -EIO;
    }

    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();
    baro_ring_reset();
    LOG_INF("Refilling FIFOs (1.6s)...");
    k_sleep(K_MSEC(1600));
    return ret;  
}

/* ======================================================================
 * LIGHT-MOTION (ANY-MOTION) TRIGGER CONTROL -- FSM API preserved.
 * Only the any-motion FEATURE is toggled; LOW-G stays armed always.
 * ====================================================================== */
void sensors_enable_motion_trigger(void)
{
    uint8_t sens = BMI2_ANY_MOTION;
    int8_t rslt = bmi270_legacy_sensor_enable(&sens, 1, &bmi_dev_ctx);
    if (rslt != BMI2_OK) {
        LOG_ERR("Failed to enable any-motion: %d", rslt);
    } else {
        LOG_INF("IMU light motion trigger enabled.");
    }
}

void sensors_disable_motion_trigger(void)
{
    uint8_t sens = BMI2_ANY_MOTION;
    int8_t rslt = bmi270_legacy_sensor_disable(&sens, 1, &bmi_dev_ctx);
    if (rslt != BMI2_OK) {
        LOG_ERR("Failed to disable any-motion: %d", rslt);
    } else {
        LOG_INF("IMU light motion trigger disabled.");
    }
}

/* ======================================================================
 * SENSOR THREAD: turns INT1 pulses into zbus events, keeps the
 * pressure FIFO fresh while idle.
 * ====================================================================== */
static void sensor_thread_fn(void *a, void *b, void *c)
{
    uint16_t int_status;
    struct bracelet_event event;

    while (1) {
#if IS_ENABLED(CONFIG_APP_COLLECT_DATA)
        /* Collection build: drain the baro FIFO into the ring every 400 ms
         * (< the ~640 ms fill time) so we always hold a fresh rolling
         * history and never hit the stop-on-full freeze. */
        if (k_sem_take(&bmi_irq_sem, K_MSEC(400)) != 0) {
            if (atomic_get(&sensors_ready) && !atomic_get(&capture_pending)) {
                baro_drain_to_ring();
            }
            continue;
        }
#else
        if (k_sem_take(&bmi_irq_sem, K_MSEC(600)) != 0) {
            if (atomic_get(&sensors_ready) && !atomic_get(&capture_pending)) {
                flush_pressure_fifo();
            }
            continue;
        }
#endif

        if (atomic_get(&capture_pending)) {
            continue;   /* capture in progress; ignore strays */
        }

        int_status = 0;
        if (bmi2_get_int_status(&int_status, &bmi_dev_ctx) != BMI2_OK) {
            continue;
        }

        if (int_status & BMI270_LEGACY_LOW_G_STATUS_MASK) {
            /* Harsh impact: hand off to the FSM exactly once until the
             * evaluation capture completes. */
            if (atomic_cas(&capture_pending, 0, 1)) {
                LOG_INF("LOW-G interrupt -> EVENT_IMU_HARSH_IMPACT");
                event.type = EVENT_IMU_HARSH_IMPACT;
                zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
            }
        } else if (int_status & BMI270_LEGACY_ANY_MOT_STATUS_MASK) {
            LOG_INF("ANY-MOTION interrupt -> EVENT_IMU_LIGHT_MOTION");
            event.type = EVENT_IMU_LIGHT_MOTION;
            zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
        }

        /* Latched pin hygiene: make sure it actually dropped */
        if (gpio_pin_get_dt(&bmi_int) > 0) {
            bmi2_get_int_status(&int_status, &bmi_dev_ctx);
        }
    }
}
K_THREAD_DEFINE(sensor_tid, 4096, sensor_thread_fn, NULL, NULL, NULL,
                K_PRIO_PREEMPT(6), 0, 0);

/* ======================================================================
 * INITIALIZATION
 * ====================================================================== */
int sensors_init(void)
{
    int ret;
    int8_t rslt;

    LOG_INF("[SENSORS] Initializing hardware...");

    // /* -- LDO1 -> buck converter enable (unchanged) -- */
    // const struct device *const ldo1_dev = DEVICE_DT_GET(LDO1_NODE);
    // if (!device_is_ready(ldo1_dev)) {
    //     LOG_ERR("[SENSORS] LDO1 not ready!");
    //     return -ENODEV;
    // }
    // ret = regulator_set_voltage(ldo1_dev, 1800000, 1800000);
    // if (ret == 0) {
    //     regulator_enable(ldo1_dev);
    //     LOG_INF("[SENSORS] LDO1 ON -- buck converter active.");
    // } else {
    //     LOG_ERR("[SENSORS] Failed to set LDO1 voltage: %d", ret);
    //     return ret;
    // }

    /* -- PWM -- */
    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("[SENSORS] PWM device not ready!");
        return -ENODEV;
    }

    /* -- Emergency button (unchanged) -- */
    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("[SENSORS] Button GPIO not ready!");
        return -ENODEV;
    }
    gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    LOG_INF("[SENSORS] Emergency button armed.");

    /* -- I2C bus -- */
    if (!i2c_is_ready_dt(&bmi_i2c) || !i2c_is_ready_dt(&icp_i2c)) {
        LOG_ERR("[SENSORS] I2C bus not ready");
        return -ENODEV;
    }

    /* -- ICP-20100: Mode 0 (25 Hz) continuous, pressure-only FIFO readout -- */
    LOG_INF("[SENSORS] Configuring ICP-20100 (25Hz, pressure-only FIFO)...");
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_MODE_SELECT,
                          0x08 | ICP20100_FIFO_READOUT_PONLY);   /* 0x0B */
    k_sleep(K_MSEC(1200));
    flush_pressure_fifo();

    /* -- BMI270 via legacy API -- */
    bmi_dev_ctx.intf = BMI2_I2C_INTF;
    bmi_dev_ctx.read = bmi2_i2c_read;
    bmi_dev_ctx.write = bmi2_i2c_write;
    bmi_dev_ctx.delay_us = bmi2_delay_us;
    bmi_dev_ctx.intf_ptr = NULL;
    bmi_dev_ctx.read_write_len = 32;

    rslt = bmi270_legacy_init(&bmi_dev_ctx);
    if (rslt != BMI2_OK) {
        LOG_ERR("[SENSORS] bmi270_legacy_init failed: %d", rslt);
        return -EIO;
    }
    bmi2_set_adv_power_save(BMI2_DISABLE, &bmi_dev_ctx);
    k_msleep(5);

    /* INT1 pin: latched, active-high, push-pull */
    struct bmi2_int_pin_config pin_cfg = { 0 };
    pin_cfg.pin_type = BMI2_INT1;
    pin_cfg.int_latch = BMI2_INT_LATCH;
    pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    pin_cfg.pin_cfg[0].od  = BMI2_INT_PUSH_PULL;
    pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    pin_cfg.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;
    bmi2_set_int_pin_config(&pin_cfg, &bmi_dev_ctx);

    gpio_pin_configure_dt(&bmi_int, GPIO_INPUT);
    gpio_init_callback(&bmi_int_cb, bmi_isr, BIT(bmi_int.pin));
    gpio_add_callback(bmi_int.port, &bmi_int_cb);

    /* Enable accel + LOW-G (always on) + ANY-MOTION (FSM-controlled) */
    uint8_t sens_list[3] = { BMI2_ACCEL, BMI2_LOW_G, BMI2_ANY_MOTION };
    rslt = bmi270_legacy_sensor_enable(sens_list, 3, &bmi_dev_ctx);
    if (rslt != BMI2_OK) {
        LOG_ERR("[SENSORS] sensor_enable failed: %d", rslt);
        return -EIO;
    }

    /* Accel: 50 Hz / 2g -- MUST match the training data */
    struct bmi2_sens_config accel_cfg = { .type = BMI2_ACCEL };
    bmi270_legacy_get_sensor_config(&accel_cfg, 1, &bmi_dev_ctx);
    accel_cfg.cfg.acc.odr         = BMI2_ACC_ODR_50HZ;
    accel_cfg.cfg.acc.range       = BMI2_ACC_RANGE_2G;
    accel_cfg.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
    accel_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    bmi270_legacy_set_sensor_config(&accel_cfg, 1, &bmi_dev_ctx);

    /* LOW-G: the harsh-impact / fall-candidate trigger (logger values) */
    struct bmi2_sens_config low_g_cfg = { .type = BMI2_LOW_G };
    bmi270_legacy_get_sensor_config(&low_g_cfg, 1, &bmi_dev_ctx);
    low_g_cfg.cfg.low_g.threshold  = 0x04CC; // 0.6g (increase if low sensitivity)
    low_g_cfg.cfg.low_g.hysteresis = 0x0100; // dont change
    low_g_cfg.cfg.low_g.duration   = 0x0004; // fall duration (80ms)
    bmi270_legacy_set_sensor_config(&low_g_cfg, 1, &bmi_dev_ctx);

    /* ANY-MOTION: the light-motion / localization-wake trigger.
     * threshold ~0.15 g, duration ~60 ms (matches old Zephyr-driver
     * values; tune to taste). */
    struct bmi2_sens_config any_mot_cfg = { .type = BMI2_ANY_MOTION };
    bmi270_legacy_get_sensor_config(&any_mot_cfg, 1, &bmi_dev_ctx);
    any_mot_cfg.cfg.any_motion.threshold = 0x136;  /* ~0.15 g (1LSB=0.49mg) */
    any_mot_cfg.cfg.any_motion.duration  = 3;      /* 3 x 20ms = 60 ms      */
    any_mot_cfg.cfg.any_motion.select_x  = 1;
    any_mot_cfg.cfg.any_motion.select_y  = 1;
    any_mot_cfg.cfg.any_motion.select_z  = 1;
    bmi270_legacy_set_sensor_config(&any_mot_cfg, 1, &bmi_dev_ctx);

    /* Map BOTH features to INT1 */
    struct bmi2_sens_int_config feat_ints[2] = {
        { .type = BMI2_LOW_G,   .hw_int_pin = BMI2_INT1 },
        { .type = BMI2_ANY_MOTION, .hw_int_pin = BMI2_INT1 },
    };
    bmi270_legacy_map_feat_int(feat_ints, 2, &bmi_dev_ctx);

    configure_bmi_fifo();

    /* Let the FIFO fill before accepting interrupts, then arm safely */
    LOG_INF("[SENSORS] Filling FIFO (1.6s) before arming...");
    k_sleep(K_MSEC(1600));
    rearm_bmi_interrupt();

    atomic_set(&sensors_ready, 1);
    LOG_INF("[SENSORS] All hardware initialised (low-g + any-motion armed).");
    return 0;
}


/* Returns approx battery level from voltage for current 3.7V battery*/
static int battery_mv_to_pct(int32_t mv)
{
    static const struct {
        int32_t mv;
        int pct;
    } lut[] = {
        {4200, 100},
        {4100, 90},
        {4000, 80},
        {3920, 70},
        {3850, 60},
        {3790, 50},
        {3730, 40},
        {3670, 30},
        {3600, 20},
        {3500, 10},
        {3300, 0},
    };

    if (mv >= lut[0].mv) return 100;
    if (mv <= lut[ARRAY_SIZE(lut) - 1].mv) return 0;

    for (size_t i = 0; i < ARRAY_SIZE(lut) - 1; i++) {
        if (mv <= lut[i].mv && mv >= lut[i + 1].mv) {
            int32_t mv_hi = lut[i].mv;
            int32_t mv_lo = lut[i + 1].mv;
            int pct_hi = lut[i].pct;
            int pct_lo = lut[i + 1].pct;
            return pct_lo + (mv - mv_lo) * (pct_hi - pct_lo) / (mv_hi - mv_lo);
        }
    }

    return 0;
}

/* Public function used to get the battery level */
int get_battery_level(void)
{
    struct sensor_value vbat;
    struct sensor_value chg_status;
    struct sensor_value vbus_status;
    struct sensor_value chg_error;
    int ret;
    int32_t mv;

    ret = sensor_sample_fetch(charger_dev);
    if (ret) {
        LOG_ERR("Battery sample fetch failed: %d", ret);
        return 0;
    }

    ret = sensor_channel_get(charger_dev, SENSOR_CHAN_GAUGE_VOLTAGE, &vbat);
    if (ret) {
        LOG_ERR("Battery voltage read failed: %d", ret);
        return 0;
    }

    ret = sensor_channel_get(charger_dev, SENSOR_CHAN_NPM13XX_CHARGER_STATUS, &chg_status);
    if (ret) {
        LOG_WRN("Charger status read failed: %d", ret);
        chg_status.val1 = -1;
    }

    ret = sensor_channel_get(charger_dev, SENSOR_CHAN_NPM13XX_CHARGER_VBUS_STATUS, &vbus_status);
    if (ret) {
        LOG_WRN("VBUS status read failed: %d", ret);
        vbus_status.val1 = -1;
    }

    ret = sensor_channel_get(charger_dev, SENSOR_CHAN_NPM13XX_CHARGER_ERROR, &chg_error);
    if (ret) {
        LOG_WRN("Charger error read failed: %d", ret);
        chg_error.val1 = -1;
    }

    mv = (vbat.val1 * 1000) + (vbat.val2 / 1000);

    LOG_INF("Battery: %d mV | status_bits=0x%02x | vbus_bits=0x%02x | error_bits=0x%02x",
        mv, chg_status.val1, vbus_status.val1, chg_error.val1);

    return battery_mv_to_pct(mv);
}
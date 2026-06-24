#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/zbus/zbus.h>
#include <stdio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

#include "events.h"
#include "sensors.h"

LOG_MODULE_REGISTER(sensors_system, LOG_LEVEL_INF);


/* Devicetree Bindings */
#define BMI270_NODE    DT_NODELABEL(bmi270)
#define ICP201XX_NODE  DT_NODELABEL(icp20100)
#define BUTTON_NODE    DT_ALIAS(sw0) 

#define PWM_CTLR_NODE       DT_NODELABEL(pwm0)
#define LDO1_NODE           DT_NODELABEL(npm1300_ldo1)
#define LDO2_NODE           DT_NODELABEL(npm1300_ldo2)

static const struct device *const pwm_dev = DEVICE_DT_GET(PWM_CTLR_NODE);

// Channel assignments for LED & Buzzer (PWM0: ch0=red, ch1=green, ch2=blue, ch3=buzzer)
#define PWM_CH_RED     0U
#define PWM_CH_GREEN   1U
#define PWM_CH_BLUE    2U
#define PWM_CH_BUZZER  3U

// Unified period: MUST be the same for all channels on nRF PWM hardware.
// 4000 Hz = buzzer resonant frequency. LEDs at 50% DC look solid at this freq.
#define PWM_UNIFIED_PERIOD  PWM_HZ(4000)
#define PWM_LED_PULSE       (PWM_UNIFIED_PERIOD / 2U)
#define PWM_BUZZ_PULSE      (PWM_UNIFIED_PERIOD / 2U)

static const struct device *bmi_dev = DEVICE_DT_GET(BMI270_NODE);
static const struct device *baro_dev = DEVICE_DT_GET(ICP201XX_NODE);
static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(BUTTON_NODE, gpios);

/* Trigger structures for the IMU */
static struct sensor_trigger imu_motion_trig;
static struct sensor_trigger imu_shock_trig;
static struct gpio_callback button_cb_data;

/* ======================================================================
 * INTERRUPT SERVICE ROUTINES (ISRs)
 * ====================================================================== */

static void button_pressed_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    /* Keep ISRs fast: Just pack the event and fire it onto ZBUS */
    struct bracelet_event event = { .type = EVENT_BUTTON_PRESSED };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

static void imu_trigger_handler(const struct device *dev, const struct sensor_trigger *trig)
{
    struct bracelet_event event;

    if (trig->type == SENSOR_TRIG_MOTION) {
        event.type = EVENT_IMU_LIGHT_MOTION;
    } 
    else if (trig->type == SENSOR_TRIG_DELTA) {
        event.type = EVENT_IMU_HARSH_IMPACT;
    }

    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

// Runs at POST_KERNEL priority 85, before the BMI270 driver at priority 90.
// Powers LDO2 (1.8V) so the BMI270 ASIC has supply before its driver inits.

static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);

    printk("\n--- BOOT PRIORITY 85: Powering up PMIC LDO2 for BMI270 ---\n");

    if (!device_is_ready(ldo2_dev)) {
        printk("WARNING: LDO2 device not ready flag set, attempting anyway...\n");
    }

    int ret = regulator_enable(ldo2_dev);
    if (ret == 0) {
        printk("SUCCESS: LDO2 enabled (1.8V rail up).\n");
    } else {
        printk("ERROR: regulator_enable(LDO2) failed: %d\n", ret);
    }

    printk("Sleeping 100ms for BMI270 ASIC to boot...\n");
    k_sleep(K_MSEC(100));
    printk("--- BOOT PRIORITY 85 COMPLETE ---\n\n");

    return 0;
}

SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

/* ======================================================================
 * PUBLIC FUNCTIONS
 * ====================================================================== */

int sensors_init(void)
{
    int ret;

    LOG_INF("[SENSORS] Initializing hardware...");

    // ── LDO1 → Buck Converter (from main-4.c) ───────────────────────
    // LDO1 at 3.3V drives the buck converter EN pin HIGH.
    // Without this, the PWM LED/buzzer circuit has no power.
    const struct device *const ldo1_dev = DEVICE_DT_GET(LDO1_NODE);
    if (!device_is_ready(ldo1_dev)) {
        LOG_ERR("[SENSORS] LDO1 not ready!");
        return -ENODEV;
    }
    ret = regulator_set_voltage(ldo1_dev, 3300000, 3300000);
    if (ret == 0) {
        regulator_enable(ldo1_dev);
        LOG_INF("[SENSORS] LDO1 ON at 3.3V — buck converter active.");
    } else {
        LOG_ERR("[SENSORS] Failed to set LDO1 voltage: %d", ret);
        return ret;
    }

    // ── PWM device check (from main-4.c) ────────────────────────────
    if (!device_is_ready(pwm_dev)) {
        LOG_ERR("[SENSORS] PWM device not ready!");
        return -ENODEV;
    }
    LOG_INF("[SENSORS] PWM controller ready.");

    // ── Emergency Button (same logic as before, error code from main-4.c) ──
    if (!gpio_is_ready_dt(&button)) {
        LOG_ERR("[SENSORS] Button GPIO not ready!");
        return -ENODEV;
    }
    ret = gpio_pin_configure_dt(&button, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("[SENSORS] Failed to configure button: %d", ret);
        return ret;
    }
    ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret < 0) {
        LOG_ERR("[SENSORS] Failed to configure button interrupt: %d", ret);
        return ret;
    }
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);
    LOG_INF("[SENSORS] Emergency button armed.");

    // ── Barometer (from main-3.c) ────────────────────────────────────
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("[SENSORS] Barometer (ICP20100) not ready!");
        return -ENODEV;
    }
    LOG_INF("[SENSORS] Barometer ready.");

    // ── BMI270 IMU (from main-2.c) ───────────────────────────────────
    if (!device_is_ready(bmi_dev)) {
        LOG_ERR("[SENSORS] BMI270 not ready, skipping trigger setup");
    } else {
        LOG_INF("[SENSORS] DEBUG: IMU READY, Triggering...");
        /* Configure chip FIRST, then arm triggers */
        struct sensor_value full_scale    = { .val1 = 2,   .val2 = 0 };
        struct sensor_value sampling_freq = { .val1 = 100, .val2 = 0 };
        struct sensor_value oversampling  = { .val1 = 1,   .val2 = 0 };

        sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,        &full_scale);
        sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING,      &oversampling);
        sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,&sampling_freq);

        /* Arm triggers ONCE, AFTER configuration */
        imu_motion_trig.type = SENSOR_TRIG_MOTION;
        imu_motion_trig.chan = SENSOR_CHAN_ACCEL_XYZ;
        sensor_trigger_set(bmi_dev, &imu_motion_trig, imu_trigger_handler);

        imu_shock_trig.type = SENSOR_TRIG_DELTA;
        imu_shock_trig.chan = SENSOR_CHAN_ACCEL_XYZ;
        sensor_trigger_set(bmi_dev, &imu_shock_trig, imu_trigger_handler);

        LOG_INF("[SENSORS] BMI270 configured and triggers armed.");
    }
    // Configure accel: 2g full-scale, 100Hz, oversampling=1
    // Taken directly from main-2.c proven working configuration.
    struct sensor_value full_scale  = { .val1 = 2,   .val2 = 0 };
    struct sensor_value sampling_freq = { .val1 = 100, .val2 = 0 };
    struct sensor_value oversampling  = { .val1 = 1,   .val2 = 0 };

    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ,
                    SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    // Arm triggers
    imu_motion_trig.type = SENSOR_TRIG_MOTION;
    imu_motion_trig.chan = SENSOR_CHAN_ACCEL_XYZ;
    sensor_trigger_set(bmi_dev, &imu_motion_trig, imu_trigger_handler);

    imu_shock_trig.type = SENSOR_TRIG_DELTA;
    imu_shock_trig.chan = SENSOR_CHAN_ACCEL_XYZ;
    sensor_trigger_set(bmi_dev, &imu_shock_trig, imu_trigger_handler);

    LOG_INF("[SENSORS] BMI270 configured and triggers armed.");
    LOG_INF("[SENSORS] All hardware initialised successfully.");
    return 0;
}

int sensors_collect_ml_window(float *ml_data_buffer, int max_features)
{
    struct sensor_value acc[3], gyr[3], press, temp;
    int index = 0;

    /* Example: 50Hz sampling (20ms period) for 2 seconds = 100 loops */
    int num_samples = 100; 
    
    printf("[SENSORS] Collecting %d seconds of ML Data...\n", (num_samples * 20) / 1000);

    for (int i = 0; i < num_samples; i++) {
        /* Ensure we don't overflow the buffer */
        if (index + 8 > max_features) {
            break;
        }

        /* Fetch data from physical chips */
        sensor_sample_fetch(bmi_dev);
        sensor_sample_fetch(baro_dev);

        /* Read IMU */
        sensor_channel_get(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
        sensor_channel_get(bmi_dev, SENSOR_CHAN_GYRO_XYZ, gyr);
        
        /* Read Baro */
        sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press);
        sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

        /* Flatten all data into the 1D ML Buffer */
        ml_data_buffer[index++] = (float)sensor_value_to_double(&acc[0]);
        ml_data_buffer[index++] = (float)sensor_value_to_double(&acc[1]);
        ml_data_buffer[index++] = (float)sensor_value_to_double(&acc[2]);
        ml_data_buffer[index++] = (float)sensor_value_to_double(&gyr[0]);
        ml_data_buffer[index++] = (float)sensor_value_to_double(&gyr[1]);
        ml_data_buffer[index++] = (float)sensor_value_to_double(&gyr[2]);
        ml_data_buffer[index++] = (float)sensor_value_to_double(&press);
        ml_data_buffer[index++] = (float)sensor_value_to_double(&temp);

        /* Sleep to maintain exactly 50Hz sampling rate */
        k_sleep(K_MSEC(20));
    }

    printf("[SENSORS] ML Data collection complete.\n");
    return index; /* Return the number of features populated */
}

// ── LED & BUZZER CONTROL FUNCTIONS ───────────────────────────────────
// All 4 PWM channels on nRF share ONE period register.
// Period is fixed at PWM_UNIFIED_PERIOD (4000 Hz) for buzzer compatibility.
// LEDs at 50% duty cycle at 4000 Hz appear as solid light (no flicker).

// Color parameter: pass a bitmask of the defines below.
// Examples:  sensors_led_on(LED_RED)
//            sensors_led_on(LED_RED | LED_BLUE)   -> magenta
//            sensors_led_on(LED_RED | LED_GREEN | LED_BLUE) -> white

#define LED_RED   BIT(0)
#define LED_GREEN BIT(1)
#define LED_BLUE  BIT(2)

void sensors_led_on(uint8_t color)
{
    const uint32_t period = PWM_UNIFIED_PERIOD;
    const uint32_t pulse  = PWM_LED_PULSE;

    if (color & LED_RED) {
        pwm_set(pwm_dev, PWM_CH_RED,   period, pulse, PWM_POLARITY_NORMAL);
    }
    if (color & LED_GREEN) {
        pwm_set(pwm_dev, PWM_CH_GREEN, period, pulse, PWM_POLARITY_NORMAL);
    }
    if (color & LED_BLUE) {
        pwm_set(pwm_dev, PWM_CH_BLUE,  period, pulse, PWM_POLARITY_NORMAL);
    }
}

void sensors_led_off(uint8_t color)
{
    const uint32_t period = PWM_UNIFIED_PERIOD;

    if (color & LED_RED) {
        pwm_set(pwm_dev, PWM_CH_RED,   period, 0U, PWM_POLARITY_NORMAL);
    }
    if (color & LED_GREEN) {
        pwm_set(pwm_dev, PWM_CH_GREEN, period, 0U, PWM_POLARITY_NORMAL);
    }
    if (color & LED_BLUE) {
        pwm_set(pwm_dev, PWM_CH_BLUE,  period, 0U, PWM_POLARITY_NORMAL);
    }
}

void sensors_buzzer_on(void)
{
    pwm_set(pwm_dev, PWM_CH_BUZZER, PWM_UNIFIED_PERIOD,
            PWM_BUZZ_PULSE, PWM_POLARITY_NORMAL);
}

void sensors_buzzer_off(void)
{
    pwm_set(pwm_dev, PWM_CH_BUZZER, PWM_UNIFIED_PERIOD,
            0U, PWM_POLARITY_NORMAL);
}
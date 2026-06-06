#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/zbus/zbus.h>
#include <stdio.h>

#include "events.h"
#include "sensors.h"

/* Devicetree Bindings */
#define BMI270_NODE    DT_NODELABEL(bmi270)
#define ICP201XX_NODE  DT_NODELABEL(icp20100)
#define BUTTON_NODE    DT_ALIAS(sw0) 

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
    else if (trig->type == SENSOR_TRIG_SHOCK || trig->type == SENSOR_TRIG_DELTA) {
        event.type = EVENT_IMU_HARSH_IMPACT;
    }

    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

/* ======================================================================
 * PUBLIC FUNCTIONS
 * ====================================================================== */

int sensors_init(void)
{
    printf("[SENSORS] Initializing Hardware Triggers...\n");

    /* 1. Setup Emergency Button */
    if (!gpio_is_ready_dt(&button)) {
        return -ENODEV;
    }
    gpio_pin_configure_dt(&button, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&button_cb_data, button_pressed_isr, BIT(button.pin));
    gpio_add_callback(button.port, &button_cb_data);

    /* 2. Setup Barometer (Verify presence, but NO interrupts) */
    if (!device_is_ready(baro_dev)) {
        printf("[ERROR] Barometer not ready!\n");
        return -ENODEV;
    }

    /* 3. Setup IMU (BMI270) */
    if (!device_is_ready(bmi_dev)) {
        printf("[ERROR] BMI270 not ready!\n");
        return -ENODEV;
    }

    /* Arm Light Movement (Any-Motion) */
    imu_motion_trig.type = SENSOR_TRIG_MOTION;
    imu_motion_trig.chan = SENSOR_CHAN_ACCEL_XYZ;
    sensor_trigger_set(bmi_dev, &imu_motion_trig, imu_trigger_handler);

    /* Arm Harsh Impact (High-G / Shock) */
    imu_shock_trig.type = SENSOR_TRIG_SHOCK; 
    imu_shock_trig.chan = SENSOR_CHAN_ACCEL_XYZ;
    sensor_trigger_set(bmi_dev, &imu_shock_trig, imu_trigger_handler);

    printf("[SENSORS] Hardware Armed Successfully.\n");
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
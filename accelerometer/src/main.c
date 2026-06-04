/*
 * Final BMI270 Implementation for Bracelet Tracker
 * Configures IMU for 100Hz and provides high-precision data
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <math.h>
#include <stdio.h>

#define BMI270_NODE DT_NODELABEL(bmi270)

/*
 * HARDWARE INTERRUPT HANDLER (Commented for threshold tuning)
 */
/*
static void bmi270_interrupt_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
    struct sensor_value acc[3];
    
    if (sensor_sample_fetch(dev) < 0) {
        printf("Interrupt sample fetch failed!\n");
        return;
    }
    
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
    
    double ax = sensor_value_to_double(&acc[0]);
    double ay = sensor_value_to_double(&acc[1]);
    double az = sensor_value_to_double(&acc[2]);
    double magnitude = sqrt(ax*ax + ay*ay + az*az);
    
    // TODO: Tune these threshold values based on testing
    #define LIGHT_MOVEMENT_THRESHOLD 1.5   // m/s^2 - light movement detection
    #define HARSH_FALL_THRESHOLD 20.0      // m/s^2 - harsh fall detection
    
    if (magnitude > HARSH_FALL_THRESHOLD) {
        printf("HARSH FALL DETECTED! Mag: %.2f m/s^2\n", magnitude);
        // Trigger emergency alert
    } else if (magnitude > LIGHT_MOVEMENT_THRESHOLD) {
        printf("Light movement detected. Mag: %.2f m/s^2\n", magnitude);
        // Log movement event
    }
}
*/

int main(void)
{
    const struct device *const dev = DEVICE_DT_GET(BMI270_NODE);
    struct sensor_value acc[3], gyr[3];
    struct sensor_value full_scale, sampling_freq, oversampling;

    if (!device_is_ready(dev)) {
        printf("Device %s is not ready. Is LDO2 on? Check SW1 and PMIC.\n", dev->name);
        return 0;
    }

    printf("BMI270 initialized successfully on %s\n", dev->name);

    /* 1. Configure Accelerometer (2G, 100Hz) */
    full_scale.val1 = 2;   /* 2G */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100;
    sampling_freq.val2 = 0;
    oversampling.val1 = 1;
    oversampling.val2 = 0;

    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    /* 2. Configure Gyroscope (500dps, 100Hz) */
    full_scale.val1 = 500; /* 500 degrees/sec */
    full_scale.val2 = 0;
    
    sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    printf("IMU configured. Starting data stream...\n");

    /*
    // Configure accelerometer threshold interrupt
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_THRESHOLD,
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };
    
    struct sensor_value thresh = {
        .val1 = 1,  // TODO: Tune threshold (0-16 for 2G range)
        .val2 = 500000  // Fractional part
    };
    
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_THRESH_LOW, &thresh);
    sensor_trigger_set(dev, &trig, bmi270_interrupt_handler);
    printf("Interrupt handler registered.\n");
    */

    while (1) {
        /* Period set to 10ms for 100Hz consistency */
        k_sleep(K_MSEC(10));

        if (sensor_sample_fetch(dev) < 0) {
            printf("Sample fetch failed!\n");
            continue;
        }

        sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
        sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);

        /* Calculate Magnitude for your FSM logic */
        double ax = sensor_value_to_double(&acc[0]);
        double ay = sensor_value_to_double(&acc[1]);
        double az = sensor_value_to_double(&acc[2]);
        double magnitude = sqrt(ax*ax + ay*ay + az*az);

        /* Print for verification. In the final app, this moves to ZBUS */
        printf("Mag: %.2f m/s^2 | GX: %d.%06d\n", magnitude, gyr[0].val1, gyr[0].val2);
    }
    return 0;
}
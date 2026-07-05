#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>

#define BMI270_NODE DT_NODELABEL(bmi270)
#define ICP20100_NODE DT_NODELABEL(icp20100)
#define LDO2_NODE DT_NODELABEL(npm1300_ldsw2)

/* =====================================================================
 * THE PRE-MAIN BOOT SEQUENCE (Priority 85)
 * ===================================================================== */
static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);

    if (device_is_ready(ldo2_dev)) {
        regulator_enable(ldo2_dev);
    }
    
    /* Allow BMI270 ASIC to boot before priority 90 sensor init */
    k_sleep(K_MSEC(100));
    return 0;
}
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

/* =====================================================================
 * MAIN APPLICATION
 * ===================================================================== */
int main(void)
{
    const struct device *const bmi_dev = DEVICE_DT_GET(BMI270_NODE);
    const struct device *const icp_dev = DEVICE_DT_GET(ICP20100_NODE);

    struct sensor_value acc[3];
    struct sensor_value pressure;

    /* Give the user 5 seconds to connect their serial monitor */
    k_sleep(K_SECONDS(5));

    if (!device_is_ready(bmi_dev) || !device_is_ready(icp_dev)) {
        printf("ERROR: Sensors failed to initialize. Check hardware.\n");
        return 0;
    }

    /* Configure BMI270 */
    struct sensor_value full_scale = { .val1 = 2, .val2 = 0 };
    struct sensor_value oversampling = { .val1 = 1, .val2 = 0 };
    struct sensor_value sampling_freq = { .val1 = 100, .val2 = 0 }; // Hardware runs at 100Hz, we poll at 50Hz

    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    /* Print CSV Header */
    printf("timestamp,acc_x,acc_y,acc_z,pressure\n");

    int64_t start_time = k_uptime_get();

    /* 50Hz Polling Loop (20ms) */
    while (1) {
        int64_t current_time = k_uptime_get();
        int64_t elapsed_ms = current_time - start_time;

        /* Fetch from both sensors */
        if (sensor_sample_fetch(bmi_dev) == 0 && sensor_sample_fetch(icp_dev) == 0) {
            
            sensor_channel_get(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
            sensor_channel_get(icp_dev, SENSOR_CHAN_PRESS, &pressure);

            /* Convert sensor values to standard doubles */
            double x = sensor_value_to_double(&acc[0]);
            double y = sensor_value_to_double(&acc[1]);
            double z = sensor_value_to_double(&acc[2]);
            
            /* Zephyr outputs pressure in kPa. Multiply by 10 to get hPa */
            double p_hpa = sensor_value_to_double(&pressure) * 10.0;

            /* Print exact CSV format */
            printf("%lld,%.2f,%.2f,%.2f,%.2f\n", elapsed_ms, x, y, z, p_hpa);
        }

        /* Sleep exactly 20ms */
        k_sleep(K_MSEC(20)); 
    }
    
    return 0;
}
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>

#define BMI270_NODE DT_NODELABEL(bmi270)
#define LDO2_NODE DT_NODELABEL(npm1300_ldo2)

/* =====================================================================
 * THE PRE-MAIN BOOT SEQUENCE
 * Runs at Priority 85. 
 * I2C/PMIC is fully ready (Pri 50-70). Sensor Driver runs next (Pri 90).
 * ===================================================================== */
static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);

    /* printk is used here because printf is not always available before main() */
    printk("\n--- BOOT PRIORITY 85: Powering up PMIC LDO2 ---\n");

    if (!device_is_ready(ldo2_dev)) {
        printk("WARNING: LDO2 device still not ready!\n");
        /* We will attempt to enable it anyway, sometimes the state flag lags */
    }

    int ret = regulator_enable(ldo2_dev);
    if (ret == 0) {
        printk("SUCCESS: LDO2 regulator enabled over I2C.\n");
    } else {
        printk("ERROR: regulator_enable failed with code: %d\n", ret);
    }

    printk("Sleeping 100ms for BMI270 ASIC to boot...\n");
    k_sleep(K_MSEC(100));
    printk("--- BOOT PRIORITY 85 COMPLETE ---\n\n");

    return 0;
}

/* Inject into POST_KERNEL boot phase at Priority 85 */
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

/* =====================================================================
 * MAIN APPLICATION
 * ===================================================================== */
int main(void)
{
    const struct device *const bmi_dev = DEVICE_DT_GET(BMI270_NODE);
    struct sensor_value acc[3];

    printf("\n*** Booting Main Application ***\n");

    if (!device_is_ready(bmi_dev)) {
        printf("ERROR: BMI270 Driver failed to initialize during priority 90.\n");
        return 0;
    }
    
    printf("SUCCESS: Firmware loaded. BMI270 is completely ready!\n");

    /* Configure the sensor */
    struct sensor_value full_scale, sampling_freq, oversampling;
    full_scale.val1 = 2;            
    full_scale.val2 = 0;
    sampling_freq.val1 = 100;       
    sampling_freq.val2 = 0;
    oversampling.val1 = 1;          
    oversampling.val2 = 0;

    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    /* Polling Loop */
    while (1) {
        if (sensor_sample_fetch(bmi_dev) < 0) {
            printf("Failed to fetch sample.\n");
        } else {
            sensor_channel_get(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
                printf("Accel [m/s^2] -> X: %d.%06d | Y: %d.%06d | Z: %d.%06d\n",
                   acc[0].val1, acc[0].val2 < 0 ? -acc[0].val2 : acc[0].val2,
                   acc[1].val1, acc[1].val2 < 0 ? -acc[1].val2 : acc[1].val2,
                   acc[2].val1, acc[2].val2 < 0 ? -acc[2].val2 : acc[2].val2);
        }
        
        k_sleep(K_MSEC(250)); 
    }
    
    return 0;
}
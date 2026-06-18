#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/init.h> /* Required for SYS_INIT */
#include <stdio.h>

#define BMI270_NODE DT_NODELABEL(bmi270)
#define LDO2_NODE DT_NODELABEL(npm1300_ldo2)

/* =====================================================================
 * 1. THE PRE-MAIN BOOT SEQUENCE
 * This runs automatically during Zephyr's boot sequence, before main()
 * ===================================================================== */
static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);

    if (!device_is_ready(ldo2_dev)) {
        return -ENODEV;
    }

    /* Turn on the power to the BMI270 */
    regulator_enable(ldo2_dev);

    /* Give the BMI270 10ms to physically wake up before the sensor driver runs */
    k_sleep(K_MSEC(10));

    return 0; /* Boot sequence continues normally */
}

/* * Inject our function into the POST_KERNEL boot phase at Priority 75.
 * (Sensor drivers default to Priority 90, so this guarantees we run first!)
 */
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 75);


/* =====================================================================
 * 2. YOUR MAIN APPLICATION
 * By the time we get here, the sensor is powered and the firmware is loaded
 * ===================================================================== */
int main(void)
{
    const struct device *const bmi_dev = DEVICE_DT_GET(BMI270_NODE);
    struct sensor_value acc[3];

    printf("Booting system...\n");

    /* Check if our SYS_INIT trick worked and the driver initialized */
    if (!device_is_ready(bmi_dev)) {
        printf("ERROR: BMI270 Driver failed to initialize.\n");
        return 0;
    }
    
    printf("SUCCESS: Firmware loaded. BMI270 is ready!\n");

    /* Simple Data Loop */
    while (1) {
        if (sensor_sample_fetch(bmi_dev) < 0) {
            printf("Failed to fetch sample.\n");
        } else {
            sensor_channel_get(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
            
            printf("Accel -> X: %d.%06d | Y: %d.%06d | Z: %d.%06d\n",
                   acc[0].val1, acc[0].val2,
                   acc[1].val1, acc[1].val2,
                   acc[2].val1, acc[2].val2);
        }
        
        k_sleep(K_MSEC(250)); 
    }
    
    return 0;
}
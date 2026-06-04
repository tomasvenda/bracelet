#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

/* Grab the node from your app.overlay file */
#define ICP20100_NODE DT_NODELABEL(icp20100)

int main(void)
{
    printf("\n=========================================\n");
    printf(" TDK ICP-201XX Barometer Hardware Test\n");
    printf("=========================================\n");

    /* Get the device binding for the Barometer */
    const struct device *const icp_dev = DEVICE_DT_GET(ICP20100_NODE);

    /* Verify the hardware driver compiled into the kernel */
    if (!device_is_ready(icp_dev)) {
        printf("[FATAL ERROR] Barometer driver is not ready!\n");
        printf("Did you forget to flick SW1 to the ON position?\n");
        return 0;
    }
    
    printf("[SUCCESS] Found %s on the I2C bus.\n\n", icp_dev->name);

    struct sensor_value pressure;
    struct sensor_value temp;

    /* The Polling Loop */
    while (1) {
        /* Fetch the data from the physical chip registers */
        if (sensor_sample_fetch(icp_dev) < 0) {
            printf("[ERROR] Failed to fetch sample from sensor. Is the switch still ON?\n");
            k_sleep(K_SECONDS(2));
            continue;
        }

        /* Read the specific channels */
        sensor_channel_get(icp_dev, SENSOR_CHAN_PRESS, &pressure);
        sensor_channel_get(icp_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

        /* Convert the Zephyr fixed-point struct into standard floats for printing */
        printf("Pressure: %.3f kPa  |  Temperature: %.2f °C\n",
               sensor_value_to_double(&pressure),
               sensor_value_to_double(&temp));

        k_sleep(K_MSEC(1000));
    }

    return 0;
}
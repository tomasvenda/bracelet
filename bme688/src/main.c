/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

int main(void)
{
    /* 1. Give the serial console 2 seconds to connect before printing anything */
    k_sleep(K_MSEC(10000));
    printf("--- BME688 Application Booting ---\n");

    const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bme680);
    struct sensor_value temp, press, humidity, gas;

    if (dev == NULL) {
        printf("ERROR: Could not get BME688 device binding.\n");
        return 0;
    }

    /* 2. Check if the device is ready. If it was deferred, it might not be. */
    if (!device_is_ready(dev)) {
        printf("ERROR: Device %s is not ready.\n", dev->name);
        printf("This usually means the I2C bus failed or the sensor has no power.\n");
        return 0;
    }

    printf("SUCCESS: Device %p name is %s is online!\n", dev, dev->name);

    while (1) {
        k_sleep(K_MSEC(1000));

        /* Fetch data and check for I2C read errors */
        if (sensor_sample_fetch(dev) < 0) {
            printf("Failed to fetch sample from BME688.\n");
            continue; 
        }

        sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
        sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);
        sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &humidity);
        sensor_channel_get(dev, SENSOR_CHAN_GAS_RES, &gas);

        printf("Temp: %d.%06d C; Press: %d.%06d kPa; Humidity: %d.%06d %%; Gas Res: %d.%06d ohms\n",
               temp.val1, temp.val2,
               press.val1, press.val2,
               humidity.val1, humidity.val2,
               gas.val1, gas.val2);
    }
    return 0;
}
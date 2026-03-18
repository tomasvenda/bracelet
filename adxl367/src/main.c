/*
 * Copyright (c) 2024
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>

int main(void)
{
    /* Get the ADXL367 device from the DeviceTree */
    const struct device *const dev = DEVICE_DT_GET_ONE(adi_adxl367);
    struct sensor_value acc[3];
    struct sensor_value full_scale, sampling_freq;

    if (!device_is_ready(dev)) {
        printf("Device %s is not ready\n", dev ? dev->name : "ADXL367");
        return 0;
    }

    printf("Device %p name is %s\n", dev, dev->name);

    /* Setting scale to 2G */
    full_scale.val1 = 2;            
    full_scale.val2 = 0;
    
    /* Setting sampling frequency to 100 Hz */
    sampling_freq.val1 = 100;       
    sampling_freq.val2 = 0;

    /* Set Sensor Attributes */
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE,
            &full_scale);
            
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY,
            &sampling_freq);

    while (1) {
        /* 10ms period, 100Hz Sampling frequency */
        k_sleep(K_MSEC(10));

        sensor_sample_fetch(dev);
        sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);

        printf("AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d;\n",
               acc[0].val1, acc[0].val2,
               acc[1].val1, acc[1].val2,
               acc[2].val1, acc[2].val2);
    }
    return 0;
}
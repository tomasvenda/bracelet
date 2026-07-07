#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>
#include <string.h>

/* Edge Impulse TinyML SDK */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

#define BMI270_NODE DT_NODELABEL(bmi270)
#define ICP20100_NODE DT_NODELABEL(icp20100)
#define LDO2_NODE DT_NODELABEL(npm1300_ldo2)

/* =====================================================================
 * EDGE IMPULSE BUFFER & CALLBACKS
 * ===================================================================== */
// This buffer holds exactly one window of data (e.g., 2 seconds of 4 axes at 50Hz)
static float feature_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

// The SDK calls this function behind the scenes to grab data for inference
int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, feature_buffer + offset, length * sizeof(float));
    return 0;
}

/* =====================================================================
 * THE PRE-MAIN BOOT SEQUENCE (Priority 85)
 * ===================================================================== */
extern "C" static int power_up_imu_during_boot(void) {
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);
    if (device_is_ready(ldo2_dev)) {
        regulator_enable(ldo2_dev);
    }
    k_sleep(K_MSEC(100)); // Allow BMI270 ASIC to boot
    return 0;
}
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

/* =====================================================================
 * MAIN APPLICATION
 * ===================================================================== */
int main(void) {
    const struct device *const bmi_dev = DEVICE_DT_GET(BMI270_NODE);
    const struct device *const icp_dev = DEVICE_DT_GET(ICP20100_NODE);

    struct sensor_value acc[3];
    struct sensor_value pressure;

    printf("\n*** Starting TinyML Fall Detection ***\n");

    if (!device_is_ready(bmi_dev) || !device_is_ready(icp_dev)) {
        printf("ERROR: Sensors failed to initialize.\n");
        return 0;
    }

    /* Configure BMI270 */
    struct sensor_value full_scale = { .val1 = 2, .val2 = 0 };
    struct sensor_value oversampling = { .val1 = 1, .val2 = 0 };
    struct sensor_value sampling_freq = { .val1 = 100, .val2 = 0 };

    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    int feature_ix = 0;

    /* Continuous 50Hz Polling & Inference Loop */
    while (1) {
        int64_t loop_start = k_uptime_get();

        if (sensor_sample_fetch(bmi_dev) == 0 && sensor_sample_fetch(icp_dev) == 0) {
            sensor_channel_get(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
            sensor_channel_get(icp_dev, SENSOR_CHAN_PRESS, &pressure);

            // Important: Order MUST match the CSV header order (X, Y, Z, Pressure)
            feature_buffer[feature_ix++] = sensor_value_to_double(&acc[0]);
            feature_buffer[feature_ix++] = sensor_value_to_double(&acc[1]);
            feature_buffer[feature_ix++] = sensor_value_to_double(&acc[2]);
            feature_buffer[feature_ix++] = sensor_value_to_double(&pressure) * 10.0; // Convert to hPa
        }

        /* Once the buffer hits 100% capacity, run the AI model */
        if (feature_ix >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
            
            signal_t signal;
            signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
            signal.get_data = &raw_feature_get_data;

            ei_impulse_result_t result = { 0 };
            
            // run_classifier executes the DSP block and the Neural Network
            EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
            
            if (res == EI_IMPULSE_OK) {
                bool fall_detected = false;

                // Loop through the output classes to find "falling"
                for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
                    if (strcmp(result.classification[ix].label, "falling") == 0) {
                        // If the AI is over 80% confident it's a fall
                        if (result.classification[ix].value > 0.80f) {
                            fall_detected = true;
                        }
                    }
                }

                if (fall_detected) {
                    printf("\n========================================\n");
                    printf(" [ALERT] FALL DETECTED! INITIATING SOS \n");
                    printf("========================================\n\n");
                    
                    // Sleep for 10 seconds to handle the emergency
                    k_sleep(K_SECONDS(10));
                    
                    // Reset the buffer entirely after a fall
                    feature_ix = 0; 
                } else {
                    // No fall? Slide the window by 50% so we overlap our next check
                    int half_buffer_size = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE / 2;
                    memmove(feature_buffer, feature_buffer + half_buffer_size, half_buffer_size * sizeof(float));
                    feature_ix = half_buffer_size;
                }
            }
        }

        /* Sleep dynamically to strictly maintain the 20ms (50Hz) loop interval */
        int64_t time_taken = k_uptime_get() - loop_start;
        if (time_taken < 20) {
            k_sleep(K_MSEC(20 - time_taken));
        }
    }
    
    return 0;
}
/* classifier.cpp -- the only C++ file in the project.
 * Wraps Edge Impulse's run_classifier() behind a C API for main.c. */

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include <zephyr/sys/printk.h>
#include <string.h>

#include "classifier.h"

extern "C" size_t fall_classifier_input_size(void)
{
    return EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
}

extern "C" size_t fall_classifier_frame_count(void)
{
    return EI_CLASSIFIER_RAW_SAMPLE_COUNT;
}

extern "C" size_t fall_classifier_axes_per_frame(void)
{
    return EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME;
}

extern "C" int fall_classifier_run(const float *features, size_t count,
                                   char *best_label, size_t label_size,
                                   float *best_score, float *anomaly_score)
{
    if (count != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printk("[EI] Feature count mismatch: got %u, model expects %u\n",
               (unsigned)count, (unsigned)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        return -1;
    }

    signal_t signal;
    int err = numpy::signal_from_buffer(features, count, &signal);
    if (err != 0) {
        printk("[EI] signal_from_buffer failed: %d\n", err);
        return -2;
    }

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);
    if (rc != EI_IMPULSE_OK) {
        printk("[EI] run_classifier failed: %d\n", (int)rc);
        return -3;
    }

    printk("[EI] DSP %d ms, NN %d ms\n",
           result.timing.dsp, result.timing.classification);

    size_t best = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        printk("[EI]   %-10s: %.3f\n",
               result.classification[i].label,
               (double)result.classification[i].value);
        if (result.classification[i].value >
            result.classification[best].value) {
            best = i;
        }
    }

    strncpy(best_label, result.classification[best].label, label_size - 1);
    best_label[label_size - 1] = '\0';
    *best_score = result.classification[best].value;

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    *anomaly_score = result.anomaly;
    printk("[EI]   anomaly   : %.3f\n", (double)result.anomaly);
#else
    *anomaly_score = 0.0f;
#endif

    return 0;
}
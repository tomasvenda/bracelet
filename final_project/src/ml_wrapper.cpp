/* ml_wrapper.cpp -- the only C++ file in the project.
 * Wraps Edge Impulse's run_classifier() behind the C API in
 * ml_wrapper.h so fsm.c can call it directly. */

#include "edge-impulse-sdk/classifier/ei_run_classifier.h"
#include <zephyr/sys/printk.h>
#include <string.h>
#include "model-parameters/model_metadata.h"
#include "model-parameters/model_variables.h"

#include "ml_wrapper.h"

#define FALL_LABEL "fall"
#define FALL_CONFIDENCE_THRESHOLD 0.75f /* Require 75% confidence to trigger */
#define FALL_ANOMALY_MAX          0.30f /* Reject if anomaly score is above this */

extern "C" size_t ml_wrapper_input_size(void)
{
    return EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
}


extern "C" int run_fall_inference(const float *features, size_t count)
{
    if (count != EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
        printk("[ML] Feature count mismatch: got %u, model expects %u\n",
               (unsigned)count, (unsigned)EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
        return -1;
    }

    signal_t signal;

    if (numpy::signal_from_buffer(features, count, &signal) != 0) {
        printk("[ML] signal_from_buffer failed\n");
        return -2;
    }

    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR rc = run_classifier(&signal, &result, false);  
    if (rc != EI_IMPULSE_OK) {
        printk("[ML] run_classifier failed: %d\n", (int)rc);
        return -3;
    }

    size_t best = 0;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        printk("[ML]   %-10s: %.3f\n",
               result.classification[i].label,
               (double)result.classification[i].value);
        if (result.classification[i].value >
            result.classification[best].value) {
            best = i;
        }
    }

    /* Evaluate the basic neural network output */
    bool is_fall = (strcmp(result.classification[best].label, FALL_LABEL) == 0);
    bool meets_confidence = (result.classification[best].value >= FALL_CONFIDENCE_THRESHOLD);
    bool is_normal = true; /* Default to true in case anomaly block is missing */

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    printk("[ML]   anomaly   : %.3f\n", (double)result.anomaly);
    /* If anomaly is present, enforce the maximum threshold */
    if (result.anomaly >= FALL_ANOMALY_MAX) {
        is_normal = false;
        if (is_fall) {
            printk("[ML] Rejecting fall due to high anomaly score.\n");
        }
    }
#endif

    /* All three conditions must be met to trigger the SOS */
    if (is_fall && meets_confidence && is_normal) {
        return 1;
    }

    /* Valid inference, but not a clean fall */
    return 0;
}
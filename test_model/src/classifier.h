/* classifier.h -- C interface to the C++ Edge Impulse classifier.
 * Keeps all C++ contained in classifier.cpp so main.c stays plain C. */
#ifndef CLASSIFIER_H
#define CLASSIFIER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Model geometry, read from the generated model-parameters headers.
 * Use these to sanity-check the firmware buffer against the model. */
size_t fall_classifier_input_size(void);        /* total floats expected  */
size_t fall_classifier_frame_count(void);       /* samples per window     */
size_t fall_classifier_axes_per_frame(void);    /* values per sample      */

/* Run inference on an interleaved feature buffer:
 *   [x0,y0,z0,dp0, x1,y1,z1,dp1, ...]  (frame-major, model axis order)
 *
 * On success returns 0 and fills:
 *   best_label    - name of the winning class (null-terminated)
 *   best_score    - its probability 0..1
 *   anomaly_score - K-means anomaly score (0 if no anomaly block)
 * All per-class scores are also printed to the console.
 * Returns negative on error (wrong count / Edge Impulse error code). */
int fall_classifier_run(const float *features, size_t count,
                        char *best_label, size_t label_size,
                        float *best_score, float *anomaly_score);

#ifdef __cplusplus
}
#endif

#endif /* CLASSIFIER_H */
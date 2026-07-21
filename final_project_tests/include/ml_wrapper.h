/* ml_wrapper.h -- C interface to the C++ Edge Impulse classifier. */
#ifndef ML_WRAPPER_H
#define ML_WRAPPER_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Total floats the deployed model expects (should be 600). Use to
 * sanity-check buffers at boot. */
size_t ml_wrapper_input_size(void);

/* Run inference on an interleaved feature window:
 *   [x0,y0,z0,dp0, x1,y1,z1,dp1, ...]
 * Returns:
 *    1  -> classified as fall
 *    0  -> classified as anything else
 *   <0  -> error (wrong feature count / EI runtime error)
 * Per-class probabilities are printed to the console. */
int run_fall_inference(const float *features, size_t count);

float ml_last_confidence(void);
float ml_last_anomaly(void);

#ifdef __cplusplus
}
#endif

#endif /* ML_WRAPPER_H */
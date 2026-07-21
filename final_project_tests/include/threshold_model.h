#ifndef THRESHOLD_MODEL_H
#define THRESHOLD_MODEL_H

#include <stddef.h>

/* Classic 3-stage threshold fall detector over the 150x4 window (50 Hz).
 * These constants ARE the model — both test builds must share them. */
#define TH_FREEFALL_G2   (0.50f * 0.50f)
#define TH_IMPACT_G2     (1.80f * 1.80f)
#define TH_IMPACT_WIN    25
#define TH_PRESSURE_HPA  0.05f
#define TH_USE_PRESSURE  1

/* Returns 1 = fall, 0 = no fall. dbg_* may be NULL. */
int run_threshold_inference(const float *f, size_t count,
                            float *dbg_ff, float *dbg_imp, float *dbg_dp);

#endif
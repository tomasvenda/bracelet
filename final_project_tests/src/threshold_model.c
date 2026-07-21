#include <math.h>
#include <stdbool.h>
#include "threshold_model.h"

int run_threshold_inference(const float *f, size_t count,
                            float *dbg_ff, float *dbg_imp, float *dbg_dp)
{
    const int frames = count / 4;
    int ff_idx = -1;
    bool impact = false;
    float min_g2 = 1e9f, max_g2 = 0.0f;

    for (int i = 0; i < frames; i++) {
        const float x = f[i*4+0], y = f[i*4+1], z = f[i*4+2];
        const float g2 = x*x + y*y + z*z;
        if (g2 < min_g2) min_g2 = g2;
        if (g2 > max_g2) max_g2 = g2;
        if (ff_idx < 0 && g2 < TH_FREEFALL_G2) ff_idx = i;
        if (ff_idx >= 0 && i > ff_idx && i <= ff_idx + TH_IMPACT_WIN &&
            g2 > TH_IMPACT_G2) impact = true;
    }
    float dp_s = 0.0f, dp_e = 0.0f;
    for (int i = 0; i < TH_IMPACT_WIN; i++) {
        dp_s += f[i*4+3];
        dp_e += f[(frames-1-i)*4+3];
    }
    const float dp = (dp_e - dp_s) / TH_IMPACT_WIN;

    if (dbg_ff)  *dbg_ff  = sqrtf(min_g2);
    if (dbg_imp) *dbg_imp = sqrtf(max_g2);
    if (dbg_dp)  *dbg_dp  = dp;

    bool fall = (ff_idx >= 0) && impact;
#if TH_USE_PRESSURE
    fall = fall && (dp > TH_PRESSURE_HPA);
#endif
    return fall ? 1 : 0;
}
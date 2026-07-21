/* Energy microbenchmark: shared capture cost + per-decision energy of
 * ML vs threshold. Bench test, UART connected, PPK logging. */

#ifdef CONFIG_APP_DETECTOR_POWER_TEST

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "sensors.h"
#include "comms.h"
#include "ml_wrapper.h"
#include "threshold_model.h"

LOG_MODULE_REGISTER(det_power, LOG_LEVEL_INF);

#define ML_BENCH_REPS   20
#define TH_BENCH_REPS   100000

static float features[600];
static volatile int sink;

void detector_power_test_run(void)
{
    comms_safe_disconnect();
    sensors_disable_motion_trigger();

    LOG_INF("TESTING THE BAROMETER");
    float p[16];
    for (int run = 0; run < 2; run++) {
        LOG_INF("[BARO] Run %d: %s when the BLUE LED turns OFF", run,
                run ? "RAISE the board fast" : "stay still");
        k_sleep(K_SECONDS(3));
        uint16_t n = sensors_debug_read_pressure(p, 16, 3000);
        LOG_INF("[BARO] n=%u", n);
        for (int i = 0; i < n; i++) LOG_INF("[BARO] %2d: %.3f hPa", i, (double)(p[i] * 10.0f));
    }
    

    LOG_INF("[DP] ===== ENERGY BENCHMARK =====");
    k_sleep(K_SECONDS(5));

    LOG_INF("[DP] CAPTURE start @ %lld ms", k_uptime_get());
    int ret = sensors_capture_fall_window(features, ARRAY_SIZE(features));
    sensors_evaluation_done();
    LOG_INF("[DP] CAPTURE end   @ %lld ms (ret=%d)", k_uptime_get(), ret);
    if (ret != 0) {
        LOG_ERR("[DP] Capture failed; benchmarking with zeroed buffer.");
    }

    k_sleep(K_SECONDS(5));

    LOG_INF("[DP] ML burst start @ %lld ms (%d reps)", k_uptime_get(), ML_BENCH_REPS);
    int64_t t0 = k_uptime_get();
    for (int i = 0; i < ML_BENCH_REPS; i++) {
        sink = run_fall_inference(features, ARRAY_SIZE(features));
    }
    int64_t ml_ms = k_uptime_get() - t0;
    LOG_INF("[DP] ML burst end @ %lld ms | total %lld ms | %.2f ms/inference",
            k_uptime_get(), ml_ms, (double)ml_ms / ML_BENCH_REPS);

    k_sleep(K_SECONDS(5));

    LOG_INF("[DP] TH burst start @ %lld ms (%d reps)", k_uptime_get(), TH_BENCH_REPS);
    t0 = k_uptime_get();
    for (int i = 0; i < TH_BENCH_REPS; i++) {
        sink = run_threshold_inference(features, ARRAY_SIZE(features),
                                       NULL, NULL, NULL);
    }
    int64_t th_ms = k_uptime_get() - t0;
    LOG_INF("[DP] TH burst end @ %lld ms | total %lld ms | %.4f ms/inference",
            k_uptime_get(), th_ms, (double)th_ms / TH_BENCH_REPS);

    LOG_INF("[DP] Done. Board at floor for PPK reference.");
    k_sleep(K_FOREVER);
}

#endif /* CONFIG_APP_DETECTOR_POWER_TEST */
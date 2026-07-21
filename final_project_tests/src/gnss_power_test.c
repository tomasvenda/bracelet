#ifdef CONFIG_APP_GNSS_POWER_TEST

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>
#include <modem/modem_info.h>
#include <net/nrf_cloud_agnss.h>
#include <nrf_modem_gnss.h>
#include <stdlib.h>
#include "comms.h"
#include "sensors.h"

LOG_MODULE_REGISTER(gnss_test, LOG_LEVEL_INF);

#define NUM_CYCLES     6
#define SLEEP_MINUTES  1
#define FIX_TIMEOUT_S  (IS_ENABLED(CONFIG_APP_GNSS_TEST_ASSISTED) ? 60 : 180)

static uint8_t agnss_buf[4096];
static size_t  agnss_len;
static K_SEM_DEFINE(agnss_sem, 0, 1);

static void agnss_cb(const uint8_t *data, size_t len)
{
    if (len <= sizeof(agnss_buf)) {
        memcpy(agnss_buf, data, len);
        agnss_len = len;
        k_sem_give(&agnss_sem);
    } else {
        LOG_ERR("[GT] A-GNSS payload too big: %u B", len);
    }
}

static int fetch_and_inject_agnss(void)
{
    char plmn[16] = {0}, cell[16] = {0}, area[16] = {0}, req[160];

    modem_info_string_get(MODEM_INFO_OPERATOR, plmn, sizeof(plmn));
    modem_info_string_get(MODEM_INFO_CELLID,   cell, sizeof(cell));
    modem_info_string_get(MODEM_INFO_AREA_CODE, area, sizeof(area));
    int mnc = atoi(&plmn[3]);          /* PLMN "23820" -> mcc 238, mnc 20 */
    plmn[3] = '\0';
    int mcc = atoi(plmn);

    snprintk(req, sizeof(req),
             "{\"agnss_request\":1,\"mcc\":%d,\"mnc\":%d,\"tac\":%lu,\"eci\":%lu}",
             mcc, mnc, strtoul(area, NULL, 16), strtoul(cell, NULL, 16));

    k_sem_reset(&agnss_sem);
    agnss_len = 0;
    comms_set_raw_response_cb(agnss_cb);
    comms_mqtt_publish(req);
    int err = k_sem_take(&agnss_sem, K_SECONDS(30));
    comms_set_raw_response_cb(NULL);
    if (err) { LOG_ERR("[GT] No A-GNSS response"); return -ETIMEDOUT; }

    LOG_INF("[GT] A-GNSS payload: %u B, injecting", agnss_len);
    return nrf_cloud_agnss_process((char *)agnss_buf, agnss_len);
}

void gnss_power_test_run(void)
{
    sensors_disable_motion_trigger();
    modem_info_init();
    comms_mqtt_disconnect();
    comms_safe_disconnect();               /* floor between cycles */

    LOG_INF("[GT] Mode: %s | %d cycles, %d min sleep, fix timeout %d s",
            IS_ENABLED(CONFIG_APP_GNSS_TEST_ASSISTED) ? "ASSISTED" : "STANDALONE",
            NUM_CYCLES, SLEEP_MINUTES, FIX_TIMEOUT_S);
    k_sleep(K_MINUTES(SLEEP_MINUTES));

    for (int i = 1; i <= NUM_CYCLES; i++) {
        LOG_INF("[GT] ===== Cycle %d WAKE @ %lld ms =====", i, k_uptime_get());
        int64_t t0 = k_uptime_get();

#if IS_ENABLED(CONFIG_APP_GNSS_TEST_ASSISTED)
        if (comms_mqtt_ensure_connected() == 0) {
            // 1. Delete old data (LTE is still active here)
            nrf_modem_gnss_nv_data_delete(NRF_MODEM_GNSS_DELETE_EPHEMERIDES);
            
            // 2. Fetch A-GNSS over MQTT (LTE MUST be active for this to work)
            if (fetch_and_inject_agnss() != 0) {
                LOG_ERR("[GT] Cycle %d: assistance failed, fixing without it", i);
            }
            
            // 3. Data injected safely. Now disconnect MQTT.
            comms_mqtt_disconnect();
            
            // 4. NOW switch to GNSS-only mode to start the actual GPS fix
            lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
        } else {
            LOG_ERR("[GT] Cycle %d: LTE failed, skipping", i);
            comms_safe_disconnect();
            k_sleep(K_MINUTES(SLEEP_MINUTES));
            continue;
        }
#else
        lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
        nrf_modem_gnss_nv_data_delete(NRF_MODEM_GNSS_DELETE_EPHEMERIDES);
#endif
        int64_t t_gnss = k_uptime_get();
        int err = do_gnss_fix_timeout(FIX_TIMEOUT_S);
        LOG_INF("[GT] Cycle %d: fix %s | TTFF %lld ms | total wake %lld ms",
                i, err == 0 ? "OK" : "TIMEOUT",
                k_uptime_get() - t_gnss, k_uptime_get() - t0);

        comms_safe_disconnect();           /* CFUN=0, back to floor */
        LOG_INF("[GT] ===== Cycle %d SLEEP @ %lld ms =====", i, k_uptime_get());
        k_sleep(K_MINUTES(SLEEP_MINUTES));
    }
    LOG_INF("[GT] Done.");
    k_sleep(K_FOREVER);
}
#endif
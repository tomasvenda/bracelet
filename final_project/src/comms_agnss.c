#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/modem_info.h>
#include <net/nrf_cloud_agnss.h>
#include <nrf_modem_gnss.h>
#include <stdlib.h>
#include <string.h>

#include "comms.h"

LOG_MODULE_REGISTER(comms_agnss, LOG_LEVEL_INF);

#define AGNSS_BUF_SIZE      8192
#define AGNSS_RESP_TIMEOUT  K_SECONDS(30)

/* Ephemerides are good for ~2-4 h. Refresh after 2 h to stay safe. */
#define AGNSS_VALID_MS      (2 * 60 * 60 * 1000LL)

static uint8_t agnss_buf[AGNSS_BUF_SIZE];
static size_t  agnss_len;
static K_SEM_DEFINE(agnss_sem, 0, 1);

static int64_t agnss_last_ok_ms;


static void agnss_cb(const uint8_t *data, size_t len)
{
    
    if (len == 0 || len > sizeof(agnss_buf)) {
        LOG_ERR("A-GNSS payload bad size: %zu B (buffer is %zu B)",
                len, sizeof(agnss_buf));
        return;   
    }

    memcpy(agnss_buf, data, len);
    agnss_len = len;
    k_sem_give(&agnss_sem);
}

/* ------------------------------------------------------------------
 * Build and publish the request, wait for the blob, inject it.
 * ------------------------------------------------------------------ */
static int fetch_and_inject_agnss(void)
{
    char plmn[16] = {0}, cell[16] = {0}, area[16] = {0}, req[160];
    int err;

    if (modem_info_string_get(MODEM_INFO_OPERATOR, plmn, sizeof(plmn)) <= 0) {
        LOG_ERR("No serving-cell PLMN -- cannot request A-GNSS");
        return -ENODATA;
    }
    modem_info_string_get(MODEM_INFO_CELLID,    cell, sizeof(cell));
    modem_info_string_get(MODEM_INFO_AREA_CODE, area, sizeof(area));

    int mnc = atoi(&plmn[3]);
    plmn[3] = '\0';
    int mcc = atoi(plmn);

    snprintk(req, sizeof(req),
             "{\"agnss_request\":1,\"mcc\":%d,\"mnc\":%d,\"tac\":%lu,\"eci\":%lu}",
             mcc, mnc, strtoul(area, NULL, 16), strtoul(cell, NULL, 16));

    k_sem_reset(&agnss_sem);
    agnss_len = 0;

    comms_set_raw_response_cb(agnss_cb);

    err = comms_mqtt_publish(req);
    if (err) {
        comms_set_raw_response_cb(NULL);
        LOG_ERR("A-GNSS request publish failed: %d", err);
        return err;
    }

    err = k_sem_take(&agnss_sem, AGNSS_RESP_TIMEOUT);
    comms_set_raw_response_cb(NULL);

    if (err) {
        LOG_ERR("No A-GNSS response within timeout");
        return -ETIMEDOUT;
    }

    err = nrf_cloud_agnss_process((const char *)agnss_buf, agnss_len);
    if (err) {
        LOG_ERR("nrf_cloud_agnss_process failed: %d", err);
        return err;
    }

    agnss_last_ok_ms = k_uptime_get();
    LOG_INF("A-GNSS injected OK.");
    return 0;
}

/* ------------------------------------------------------------------
 * Returns 0 if the modem now holds usable assistance (either freshly
 * injected or still valid from a previous fetch), negative otherwise.
 * ------------------------------------------------------------------ */
int comms_agnss_refresh_if_needed(void)
{
    int64_t now = k_uptime_get();

    if (agnss_last_ok_ms != 0 && (now - agnss_last_ok_ms) < AGNSS_VALID_MS) {
        return 0;
    }

    return fetch_and_inject_agnss();
}

/* Force a refetch on the next call */
void comms_agnss_invalidate(void)
{
    agnss_last_ok_ms = 0;
}
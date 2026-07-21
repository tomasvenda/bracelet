#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <nrf_modem_gnss.h>
#include <stdio.h>

LOG_MODULE_REGISTER(gnss_test, LOG_LEVEL_INF);

static struct nrf_modem_gnss_pvt_data_frame pvt;
static K_SEM_DEFINE(pvt_sem, 0, 1);

static int64_t start_time_ms;
static bool first_fix_reported;

/* GNSS event handler — runs in interrupt context: read the data
 * and hand off to the main thread, nothing heavier. */
static void gnss_event_handler(int event)
{
    if (event == NRF_MODEM_GNSS_EVT_PVT) {
        if (nrf_modem_gnss_read(&pvt, sizeof(pvt), NRF_MODEM_GNSS_DATA_PVT) == 0) {
            k_sem_give(&pvt_sem);
        }
    }
}

static void print_satellite_stats(const struct nrf_modem_gnss_pvt_data_frame *p)
{
    int tracked = 0;
    int used = 0;

    for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
        if (p->sv[i].sv == 0) {
            continue; /* Empty slot */
        }
        tracked++;
        if (p->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
            used++;
        }
        /* cn0 is in units of 0.1 dB-Hz */
        printk("  SV %3u  C/N0 %2u.%u dB-Hz  elev %2d  %s\n",
               p->sv[i].sv,
               p->sv[i].cn0 / 10, p->sv[i].cn0 % 10,
               p->sv[i].elevation,
               (p->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) ?
                   "[in fix]" : "");
    }

    printk("Tracking %d satellites (%d used in fix)\n", tracked, used);
}

static void print_pvt(const struct nrf_modem_gnss_pvt_data_frame *p)
{
    if (p->flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
        if (!first_fix_reported) {
            first_fix_reported = true;
            printk("\n*** FIRST FIX — TTFF: %lld s ***\n",
                   (k_uptime_get() - start_time_ms) / 1000);
        }
        printk("FIX  Lat: %.6f  Lon: %.6f  Alt: %.1f m  Acc: %.1f m\n",
               p->latitude, p->longitude,
               (double)p->altitude, (double)p->accuracy);
        printk("     Speed: %.1f m/s  Heading: %.1f  %04u-%02u-%02u %02u:%02u:%02u UTC\n",
               (double)p->speed, (double)p->heading,
               p->datetime.year, p->datetime.month, p->datetime.day,
               p->datetime.hour, p->datetime.minute, p->datetime.seconds);
    } else {
        printk("Searching... (elapsed %lld s)\n",
               (k_uptime_get() - start_time_ms) / 1000);
        if (p->flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
            printk("  (warning: GNSS not getting enough runtime)\n");
        }
    }

    print_satellite_stats(p);
    printk("\n");
}

int main(void)
{
    int err;

    printk("=== nRF9151 standalone GNSS + LNA test ===\n");

    /* 1. Bring up the modem library (modem starts in CFUN=0). */
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem library init failed: %d", err);
        return -1;
    }

    /* 2. Configure COEX0 to drive the external GNSS LNA enable pin.
     *    "1,1,1565,1586" = enable COEX0, active HIGH, for the GNSS
     *    L1 band 1565-1586 MHz. Must be sent while modem is inactive. */
    err = nrf_modem_at_printf("AT%%XCOEX0=1,1,1565,1586");
    if (err) {
        LOG_ERR("Failed to configure COEX0 for LNA: %d", err);
        LOG_ERR("LNA will stay OFF — expect weak/no satellites!");
        /* Continue anyway so you can compare with/without LNA. */
    } else {
        LOG_INF("COEX0 configured: LNA enabled during GNSS reception");
    }

    /* DIAGNOSTICS: read back what the modem actually stored, and
     * print the modem firmware version. */
    char resp[128];
    if (nrf_modem_at_cmd(resp, sizeof(resp), "AT%%XCOEX0?") == 0) {
        LOG_INF("COEX0 readback: %s", resp);
    } else {
        LOG_WRN("COEX0 readback failed");
    }
    if (nrf_modem_at_cmd(resp, sizeof(resp), "AT+CGMR") == 0) {
        LOG_INF("Modem FW: %s", resp);
    }

    char token_resp[256];

    if (nrf_modem_at_cmd(token_resp, sizeof(token_resp), "AT%%ATTESTTOKEN") == 0) {
        LOG_INF("Attestation Token: %s", token_resp);
    } else {
        LOG_WRN("Failed to retrieve attestation token");
    }

    /* 3. Register the GNSS event handler BEFORE activating GNSS. */
    err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
    if (err) {
        LOG_ERR("Failed to set GNSS event handler: %d", err);
        return -1;
    }

    /* 4. GNSS-only functional mode — no LTE, no SIM required. */
    err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
    if (err) {
        LOG_ERR("Failed to activate GNSS functional mode: %d", err);
        return -1;
    }

    /* 5. Continuous navigation: one PVT every second.
     *    (The final project uses single-fix mode with a 180 s retry;
     *    continuous mode is better for RF bring-up.) */
    err = nrf_modem_gnss_fix_interval_set(1);
    if (err) {
        LOG_ERR("Failed to set fix interval: %d", err);
        return -1;
    }

    err = nrf_modem_gnss_start();
    if (err) {
        LOG_ERR("Failed to start GNSS: %d", err);
        return -1;
    }

    start_time_ms = k_uptime_get();
    LOG_INF("GNSS started, waiting for satellites (go outdoors / near a window)...");

    while (1) {
        k_sem_take(&pvt_sem, K_FOREVER);
        print_pvt(&pvt);
    }

    return 0;
}
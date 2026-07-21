#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>

#include <nrf_modem_gnss.h>
#include <modem/modem_info.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>

#include "comms.h"

LOG_MODULE_REGISTER(comms_gnss, LOG_LEVEL_INF);

static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static bool gnss_has_fix = false;

static K_SEM_DEFINE(gnss_fix_sem, 0, 1);

static void gnss_event_handler(int event)
{
    if (event == NRF_MODEM_GNSS_EVT_PVT) {
        if (nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT) == 0) {

            int tracked = 0;
            int used = 0;
            for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
                if (last_pvt.sv[i].sv == 0) continue; 
                
                tracked++;
                bool in_fix = last_pvt.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX;
                if (in_fix) used++;
                
                printk("  SV %3u  C/N0 %2u.%u dB-Hz  elev %2d  %s\n",
                       last_pvt.sv[i].sv,
                       last_pvt.sv[i].cn0 / 10, last_pvt.sv[i].cn0 % 10,
                       last_pvt.sv[i].elevation,
                       in_fix ? "[in fix]" : "");
            }
            printk("GNSS: tracking %d satellites (%d used in fix)\n", tracked, used);

            if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
                LOG_INF("GNSS FIX ACQUIRED! Lat: %.6f, Lon: %.6f", last_pvt.latitude, last_pvt.longitude);
                gnss_has_fix = true;
                k_sem_give(&gnss_fix_sem);
            }
        }
    }
}


int do_gnss_fix(void)
{
    gnss_has_fix = false;
    k_sem_reset(&gnss_fix_sem); 

    nrf_modem_gnss_fix_interval_set(0);    /* Single fix mode */
    nrf_modem_gnss_fix_retry_set(10);     /* 10s hardware timeout */
    nrf_modem_gnss_start();

    LOG_INF("Searching for GNSS satellites (up to 3 minutes)...");
    
    int res = k_sem_take(&gnss_fix_sem, K_SECONDS(10));
    
    nrf_modem_gnss_stop();

    return (res == 0 && gnss_has_fix) ? 0 : -ETIMEDOUT;
}

int do_gnss_fix_timeout(uint16_t seconds)
{
    gnss_has_fix = false;
    k_sem_reset(&gnss_fix_sem);
    nrf_modem_gnss_fix_interval_set(0);
    nrf_modem_gnss_fix_retry_set(seconds);
    nrf_modem_gnss_start();
    int res = k_sem_take(&gnss_fix_sem, K_SECONDS(seconds + 5));
    nrf_modem_gnss_stop();
    return (res == 0 && gnss_has_fix) ? 0 : -ETIMEDOUT;
}


const struct nrf_modem_gnss_pvt_data_frame* comms_gnss_get_pvt(void)
{
    return &last_pvt;
}


int comms_gnss_init(void)
{
    int err;

    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem init failed, err %d", err);
        return -1;
    }

    err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
    if (err) {
        LOG_ERR("Failed to set GNSS event handler: %d", err);
        return -1;
    }

    err = nrf_modem_at_printf("AT%%XCOEX0=1,1,1565,1586");
    if (err) {
        LOG_ERR("Failed to configure COEX0 for LNA: %d", err);
    } else {
        LOG_INF("COEX0 configured: External LNA enabled for GNSS");
    }

    return 0;
}


/*  Add this before and after GNSS

if (mqtt_is_connected) {
        mqtt_helper_disconnect();
        k_msleep(300);
        mqtt_is_connected = false;
    }

    //before 
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);

    //after

    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_GNSS);
    // Restore LTE so the payload can be published afterwards.
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_LTE);


*/
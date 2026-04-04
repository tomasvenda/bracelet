#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <nrf_modem_gnss.h>
#include <modem/modem_info.h>
#include <modem/lte_lc.h>
#include <stdio.h>
#include <string.h>

#include <stdlib.h>

#include "app_events.h"
#include "data_manager.h"

LOG_MODULE_REGISTER(localization, LOG_LEVEL_INF);

/* Synchronization primitives */
static K_SEM_DEFINE(wifi_scan_sem, 0, 1);
static K_SEM_DEFINE(gnss_fix_sem, 0, 1);

/* Wi-Fi scan results storage */
static struct ap_data_t best_aps[MAX_APS_PER_SCAN];
static uint8_t current_ap_count = 0;

/* GNSS storage */
static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static bool gnss_has_fix = false;

/* --- WiFi Handlers --- */
static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
    char mac_string_buf[18];

    snprintf(mac_string_buf, sizeof(mac_string_buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             entry->mac[0], entry->mac[1], entry->mac[2], 
             entry->mac[3], entry->mac[4], entry->mac[5]);

    for (int i = 0; i < current_ap_count; i++) {
        if (strcmp(best_aps[i].mac, mac_string_buf) == 0) return;
    }

    if (current_ap_count < MAX_APS_PER_SCAN) {
        strcpy(best_aps[current_ap_count].mac, mac_string_buf);
        best_aps[current_ap_count].rssi = entry->rssi;
        current_ap_count++;
    } else {
        int weakest_idx = 0;
        for (int i = 1; i < MAX_APS_PER_SCAN; i++) {
            if (best_aps[i].rssi < best_aps[weakest_idx].rssi) weakest_idx = i;
        }
        if (entry->rssi > best_aps[weakest_idx].rssi) {
            strcpy(best_aps[weakest_idx].mac, mac_string_buf);
            best_aps[weakest_idx].rssi = entry->rssi;
        }
    }
}

static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
    k_sem_give(&wifi_scan_sem);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) {
        handle_wifi_scan_result(cb);
    } else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) {
        handle_wifi_scan_done(cb);
    }
}

/* --- GNSS Handler --- */
/* GNSS event handler — called by the modem on every GNSS event */
static void gnss_event_handler(int event)
{
    switch (event) {

    case NRF_MODEM_GNSS_EVT_PVT:
        /* Read the latest PVT data from the modem into our buffer */
        if (nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT) != 0) {
            LOG_ERR("Failed to read GNSS PVT data");
            return;
        }

        /* Count visible satellites and log their signal strength (CN0) */
        int svs_tracked = 0;
        for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
            /* signal != 0 means this slot contains a real satellite */
            if (last_pvt.sv[i].signal != 0) {
                LOG_INF("  SV: %2d | CN0: %4.1f dB-Hz | Signal type: %d",
                        last_pvt.sv[i].sv,
                        last_pvt.sv[i].cn0 * 0.1,   /* CN0 is stored as tenths of dB-Hz */
                        last_pvt.sv[i].signal);
                svs_tracked++;
            }
        }
        LOG_INF("Satellites tracked: %d (need 4+ for a fix)", svs_tracked);

        /* Check if the modem has computed a valid position fix */
        if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
            LOG_INF(">>> FIX ACQUIRED! Lat: %.6f, Lon: %.6f, Acc: %.1f m",
                    last_pvt.latitude,
                    last_pvt.longitude,
                    (double)last_pvt.accuracy);
            gnss_has_fix = true;
            /* Unblock the do_gnss_fix() thread that is waiting on this semaphore */
            k_sem_give(&gnss_fix_sem);
        }
        break;

    default:
        LOG_WRN("GNSS Event: Unknown event id %d", event);
        break;
    }
}

/* 
    WI-FI Scannning process starts here, takes a semaphore that is released 
    once the module confirms through a message the process succeeded in the handle_wifi_scan_done
*/
static int do_wifi_scan(void)
{
    struct net_if *iface = net_if_get_default();
    if (!iface) return -ENOENT;

    current_ap_count = 0;
    struct wifi_scan_params params = { .scan_type = WIFI_SCAN_TYPE_ACTIVE, .dwell_time_active = 50 };

    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(params))) {
        return -EIO;
    }

    k_sem_take(&wifi_scan_sem, K_SECONDS(10));
    return current_ap_count;
}

/* 
    GNSS fix process starts here. 
*/
static int do_gnss_fix(void)
{
    gnss_has_fix = false;
    nrf_modem_gnss_event_handler_set(gnss_event_handler);

    /* Power down Wi-Fi before GNSS — RF interference degrades signal */
    struct net_if *wifi_iface = net_if_get_default();
    if (wifi_iface) {
        net_if_down(wifi_iface);
    }

    /* Deactivate LTE so GNSS can have the RF front-end */
    if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE) != 0) {
        LOG_ERR("Failed to deactivate LTE");
        return -EIO;
    }

    /* EXPLICITLY activate GNSS */
    if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0) {
        LOG_ERR("Failed to activate GNSS functional mode");
        return -EIO;
    }

    nrf_modem_gnss_fix_interval_set(0);    // single-fix mode
    nrf_modem_gnss_fix_retry_set(180);     // 180s hardware timeout

    if (nrf_modem_gnss_start() != 0) {
        LOG_ERR("Failed to start GNSS");
        return -EIO;
    }

    LOG_INF("GNSS started, searching for fix...");

    int res = k_sem_take(&gnss_fix_sem, K_SECONDS(180));

    nrf_modem_gnss_stop();

    /* Deactivate GNSS cleanly before restoring LTE */
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_GNSS);

    /* Restore LTE */
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);

    /* Restore Wi-Fi */
    if (wifi_iface) {
        net_if_up(wifi_iface);
    }

    return (res == 0) ? 0 : -ETIMEDOUT;
}

/*
    Last-Resort we can approximate location with LTE cell towers. (if gnss cannot find a fix in 180 seconds)
*/
static void do_lte_cell_id(void)
{
    struct modem_param_info modem_param = {0};  // storage for modem internal parameters
    modem_info_init();                          // initialize the modem info library
    modem_info_params_init(&modem_param);       // Initialize the parameter structure
    modem_info_params_get(&modem_param);        // Ask modem for network status tokens

    // The collected tower id's will be sent to cloud: MCC = Country Code, MNC = Network Code, cellid and area_code
    char *cellid_str = modem_param.network.cellid_hex.value_string;
    uint32_t cell_id_val = (cellid_str != NULL && strlen(cellid_str) > 0) ? strtol(cellid_str, NULL, 16) : 0;

    data_manager_send_cell(modem_param.network.mcc.value, 
                           modem_param.network.mnc.value, 
                           cell_id_val, 
                           modem_param.network.area_code.value);
}

void localization_thread(void *p1, void *p2, void *p3)
{
    static struct net_mgmt_event_callback wifi_cb;
    net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler, 
                                 NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wifi_cb);

    LOG_INF("Integrated Localization Thread Started");

    while (1) {
        k_event_wait(&app_events, EVENT_USER_MOVING, false, K_FOREVER);
        
        LOG_INF("Motion detected! Starting localization waterfall...");

        /* 1. Try Wi-Fi */
        int aps = do_wifi_scan();
        if (aps >= 300) {           // 300 just while debugging gnss and lte
            LOG_INF("Wi-Fi sufficient (%d APs). Sending...", aps);
            data_manager_send_wifi(best_aps, aps);
        } else {
            LOG_INF("Wi-Fi insufficient (%d APs). Falling back to GNSS...", aps);
            
            /* 2. Try GNSS */
            if (do_gnss_fix() == 0) {
                LOG_INF("GNSS Fix obtained!");
                data_manager_send_gnss(last_pvt.latitude, last_pvt.longitude, last_pvt.accuracy);
            } else {
                LOG_INF("GNSS Timeout. Falling back to LTE Cell ID...");
                
                /* 3. Try LTE Cell ID */
                do_lte_cell_id();
            }
        }

        k_sleep(K_SECONDS(30)); /* Cooldown */
    }
}

K_THREAD_DEFINE(loc_thread_id, 4096, localization_thread, NULL, NULL, NULL, 7, 0, 0);

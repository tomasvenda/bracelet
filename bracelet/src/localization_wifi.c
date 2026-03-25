#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/wifi_mgmt.h>
#include <stdio.h>
#include <string.h>

#include "app_events.h"    
#include "data_manager.h"  

LOG_MODULE_REGISTER(wifi_module, LOG_LEVEL_INF);

static struct net_mgmt_event_callback wifi_mgmt_cb;
static K_SEM_DEFINE(scan_sem, 0, 1);

/* Buffer for our Top 10 strongest networks */
static struct ap_data_t best_aps[MAX_APS_PER_SCAN];
static uint8_t current_ap_count = 0;

static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
    char mac_string_buf[18];

    /* Format MAC address safely */
    snprintf(mac_string_buf, sizeof(mac_string_buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             entry->mac[0], entry->mac[1], entry->mac[2], 
             entry->mac[3], entry->mac[4], entry->mac[5]);

    /* Check if we already have this exact MAC address (rare, but good to filter) */
    for (int i = 0; i < current_ap_count; i++) {
        if (strcmp(best_aps[i].mac, mac_string_buf) == 0) {
            return; 
        }
    }

    /* If array isn't full, just add it */
    if (current_ap_count < MAX_APS_PER_SCAN) {
        strcpy(best_aps[current_ap_count].mac, mac_string_buf);
        best_aps[current_ap_count].rssi = entry->rssi;
        current_ap_count++;
    } 
    /* If array is full, find the weakest network and replace it if this one is stronger */
    else {
        int weakest_idx = 0;
        for (int i = 1; i < MAX_APS_PER_SCAN; i++) {
            if (best_aps[i].rssi < best_aps[weakest_idx].rssi) {
                weakest_idx = i;
            }
        }
        if (entry->rssi > best_aps[weakest_idx].rssi) {
            strcpy(best_aps[weakest_idx].mac, mac_string_buf);
            best_aps[weakest_idx].rssi = entry->rssi;
        }
    }
}

static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
    LOG_INF("Scan complete! Sending top %d networks to cloud...", current_ap_count);
    
    if (current_ap_count > 0) {
        data_manager_send_wifi_batch(best_aps, current_ap_count);
    }
    
    k_sem_give(&scan_sem);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) {
        handle_wifi_scan_result(cb);
    } else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) {
        handle_wifi_scan_done(cb);
    }
}

static int wifi_scan(void)
{
    struct net_if *iface = net_if_get_default();
    if (!iface) iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));
    if (!iface) iface = net_if_get_by_index(1); 

    if (!iface) {
        LOG_ERR("WiFi interface not found!");
        return -ENOENT;
    }

    if (!net_if_is_up(iface)) {
        net_if_up(iface);
        k_sleep(K_MSEC(500)); 
    }

    struct wifi_scan_params params = { 0 };
    params.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    params.dwell_time_active = 50;  

    /* CRITICAL: Reset the counter before starting a new scan! */
    current_ap_count = 0;

    LOG_INF("Starting WiFi scan...");
    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(struct wifi_scan_params))) {
        LOG_ERR("WiFi scan request failed");
        return -ENOEXEC;
    }

    k_sem_take(&scan_sem, K_FOREVER);
    return 0;
}

void wifi_thread_entry(void *p1, void *p2, void *p3)
{
    LOG_INF("WiFi Scanner Thread Initialized");

    net_mgmt_init_event_callback(&wifi_mgmt_cb, wifi_mgmt_event_handler, 
                                 NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wifi_mgmt_cb);

    while (1) {
        k_event_wait(&app_events, EVENT_USER_MOVING, false, K_FOREVER);
        wifi_scan();
        k_sleep(K_SECONDS(10));
    }
}

K_THREAD_DEFINE(wifi_thread_id, 2048, wifi_thread_entry, NULL, NULL, NULL, 6, 0, 0);
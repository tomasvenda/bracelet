
// Initial headers for Zephyr and standard C libraries 

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/ethernet.h>

/* Our custom headers to link the modules */
#include "app_events.h"    
#include "data_manager.h"  

// Register log module
LOG_MODULE_REGISTER(wifi_module, LOG_LEVEL_INF);

// Initialize counter for wifi scans
static uint32_t scan_result_count;

// Initialize the event callback
static struct net_mgmt_event_callback wifi_mgmt_cb;

// INitialize semaphore to signal when scan is done
static K_SEM_DEFINE(scan_sem, 0, 1);

// Callback when a single network is found
static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
    // info contains all the properties from the wifi scan and we pass it to entry for easier access
    const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
    uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];
    char ssid_print[WIFI_SSID_MAX_LEN + 1];

    scan_result_count++;

    // check to not break the buffer
    int ssid_len = MIN(entry->ssid_length, WIFI_SSID_MAX_LEN);
    // Pulling the raw bites from entry and adding null terminator
    memcpy(ssid_print, entry->ssid, ssid_len);
    ssid_print[ssid_len] = '\0';
    // Convert MAC address to human readable
    net_sprint_ll_addr_buf(entry->mac, WIFI_MAC_ADDR_LEN, mac_string_buf, sizeof(mac_string_buf));

    LOG_INF("Found network: %s | RSSI: %d", ssid_print, entry->rssi);

    /* SEND TO DATA MANAGER */
    data_manager_send_wifi_result(ssid_print, entry->rssi, mac_string_buf);
}

/* Callback when the entire scan process is finished */
static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
    k_sem_give(&scan_sem);
}

/* Route the events to the correct handler */
static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) {
        handle_wifi_scan_result(cb);
    } else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) {
        handle_wifi_scan_done(cb);
    }
}

/* Function to trigger the actual hardware scan */
static int wifi_scan(void)
{
    // Tries to find the interface with Ethernet 2 
    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));
    struct wifi_scan_params params = { 0 };

    if (!iface) {
        LOG_ERR("WiFi interface not found");
        return -ENOENT;
    }

    // Wifi scan type is active and it listens for 50 ms
    params.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    params.dwell_time_active = 50;  

    // Reset counter
    scan_result_count = 0;
    LOG_INF("Starting WiFi scan...");

    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(struct wifi_scan_params))) {
        LOG_ERR("WiFi scan request failed");
        return -ENOEXEC;
    }

    /* Wait for handle_wifi_scan_done to release this semaphore */
    k_sem_take(&scan_sem, K_FOREVER);
    LOG_INF("Scan complete. Found %d networks.", scan_result_count);
    return 0;
}

/* * ========================================================
 * THREAD ENTRY POINT
 * ========================================================
 */
void wifi_thread_entry(void *p1, void *p2, void *p3)
{
    LOG_INF("WiFi Scanner Thread Initialized");

    /* Register the callbacks once at boot */
    net_mgmt_init_event_callback(&wifi_mgmt_cb, wifi_mgmt_event_handler, 
                                 NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wifi_mgmt_cb);

    /* Main Thread Loop */
    while (1) {
        
        /* Check if the user is moving. If they are NOT moving, this thread will pause here completely (K_FOREVER) */
        k_event_wait(&app_events, EVENT_USER_MOVING, false, K_FOREVER);

        wifi_scan();

        k_sleep(K_SECONDS(10));
    }
}

// Define the thread with a stack size of 2048 bytes, priority 6, and no special options
K_THREAD_DEFINE(wifi_thread_id, 2048, wifi_thread_entry, NULL, NULL, NULL, 6, 0, 0);
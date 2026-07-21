#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>
#include <string.h>

#include "comms.h"

/* Register a unique logger name for this specific file */
LOG_MODULE_REGISTER(comms_wifi, LOG_LEVEL_INF);

#define MAX_APS_PER_SCAN 20

static struct ap_data_t best_aps[MAX_APS_PER_SCAN];
static uint32_t current_ap_count;
static uint32_t scan_result;
static struct net_mgmt_event_callback wifi_cb;

static K_SEM_DEFINE(wifi_scan_sem, 0, 1);

static const struct device *const ldo1_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldo1));

static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event) {

    case NET_EVENT_WIFI_SCAN_RESULT:
    {
        const struct wifi_scan_result *entry =
            (const struct wifi_scan_result *)cb->info;

        scan_result++;

        LOG_INF("[%02d] BSSID=%02X:%02X:%02X:%02X:%02X:%02X  RSSI=%d dBm  CH=%d  SSID=%s",
            scan_result,
            entry->mac[0],
            entry->mac[1],
            entry->mac[2],
            entry->mac[3],
            entry->mac[4],
            entry->mac[5],
            entry->rssi,
            entry->channel,
            entry->ssid);
        
        /* Save the MAC and RSSI for the MQTT payload */
        char mac_string_buf[18];
        snprintf(mac_string_buf, sizeof(mac_string_buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 entry->mac[0], entry->mac[1], entry->mac[2], 
                 entry->mac[3], entry->mac[4], entry->mac[5]);

        /* Prevent duplicates and store the MAX_APS_PER_SCAN strongest signals */
        bool duplicate = false;
        for (int i = 0; i < current_ap_count; i++) {
            if (strcmp(best_aps[i].mac, mac_string_buf) == 0) duplicate = true;
        }

        if (!duplicate) {
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

        break;
    }

    case NET_EVENT_WIFI_SCAN_DONE:
    {
        const struct wifi_status *status = (const struct wifi_status *)cb->info;

        k_sem_give(&wifi_scan_sem);
        break;
    }

    default:
        break;
    }
}


int do_wifi_scan(void){
    /* Fetch the interface. Zephyr Auto-Init created this during boot. */
    struct net_if *iface = net_if_get_wifi_sta();

    if (!iface) {
        LOG_ERR("No Wi-Fi interface found");
        return -1;
    }

    struct wifi_scan_params params = {0};

    /* Active scan sends probe requests instead of only listening. */
    params.scan_type = WIFI_SCAN_TYPE_ACTIVE;

    /* Scan both 2.4 GHz and 5 GHz. */
    params.bands =
        BIT(WIFI_FREQ_BAND_2_4_GHZ) |
        BIT(WIFI_FREQ_BAND_5_GHZ);

    /* Stay longer on each channel. */
    params.dwell_time_active = 120;
    params.dwell_time_passive = 120;

    /* Return as many APs as possible. */
    params.max_bss_cnt = 64;

    scan_result = 0;
    current_ap_count = 0;
    k_sem_reset(&wifi_scan_sem); /* drop any stale/abort-time token */

    // Sends the scan request to the driver
    int ret = net_mgmt(
        NET_REQUEST_WIFI_SCAN,
        iface,
        &params,
        sizeof(params));

    if (ret) {
        LOG_ERR("Failed to start scan (%d)", ret);
        return ret;
    }

    LOG_INF("Scanning...");

    k_sem_take(&wifi_scan_sem, K_SECONDS(20));

    return current_ap_count;
}

const struct ap_data_t* comms_wifi_get_aps(void) 
{
    return best_aps; /* Returns a read-only pointer to the array */
}

void comms_wifi_init(void)
{
    net_mgmt_init_event_callback(
        &wifi_cb,
        wifi_event_handler,
        NET_EVENT_WIFI_SCAN_RESULT |
        NET_EVENT_WIFI_SCAN_DONE);

    net_mgmt_add_event_callback(&wifi_cb);

    /* Bring the Wi-Fi rail and interface up, then straight back down. */
    if (device_is_ready(ldo1_dev)) {
        int ret = regulator_enable(ldo1_dev);
        if (ret) {
            LOG_ERR("LDO1 enable failed: %d", ret);
        } else {
            LOG_INF("LDO1 enabled.");
        }
    } else {
        LOG_ERR("LDO1 not ready!");
    }

    k_msleep(10);

    struct net_if *iface = net_if_get_wifi_sta();
    if (iface) {
        int ret = net_if_up(iface);
        LOG_INF("Wi-Fi interface up (%d).", ret);
        k_msleep(500);

        ret = net_if_down(iface);
        LOG_INF("Wi-Fi interface down (%d).", ret);
        k_msleep(100);
    } else {
        LOG_ERR("Wi-Fi interface not found!");
    }

    if (device_is_ready(ldo1_dev)) {
        int ret = regulator_disable(ldo1_dev);
        if (ret) {
            LOG_ERR("LDO1 disable failed: %d", ret);
        } else {
            LOG_INF("LDO1 disabled.");
        }
    }
}
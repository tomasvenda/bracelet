#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>

#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>

LOG_MODULE_REGISTER(wifi_diag, LOG_LEVEL_INF);

/* Callback object used by the Zephyr network manager to report Wi-Fi events. */
static struct net_mgmt_event_callback wifi_cb;

static uint32_t scan_result;    // Number of AP's found during a scan

K_SEM_DEFINE(scan_sem, 0, 1);   // Semaphore that will block the thread until the async wifi scan finishes

/*---------------------------------------------------------------
 * Wi-Fi event handler
 *
 * This function is called automatically by Zephyr whenever a
 * registered Wi-Fi management event occurs.
 *--------------------------------------------------------------*/
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

        break;
    }

    case NET_EVENT_WIFI_SCAN_DONE:
    {
        const struct wifi_status *status = (const struct wifi_status *)cb->info;

        LOG_INF("Scan done status=%d", status->status);

        k_sem_give(&scan_sem);
        break;
    }

    default:
        break;
    }
}


/*---------------------------------------------------------------
 * Start a Wi-Fi scan and wait until it completes.
 *
 * Returns:
 *   0  -> Scan completed successfully
 *   <0 -> Failed to start the scan
 *--------------------------------------------------------------*/
static int wifi_scan(struct net_if *iface)
{
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

    k_sem_take(&scan_sem, K_SECONDS(20));

    LOG_INF("Found %d access points", scan_result);

    return 0;
}

int main(void)
{
    LOG_INF("======NRF 7002 Scan Test starting...======");

    /* Fetch the interface. Zephyr Auto-Init created this during boot. */
    struct net_if *iface = net_if_get_wifi_sta();

    if (!iface) {
        LOG_ERR("No Wi-Fi interface found");
        return -1;
    }
    LOG_INF("[OK]   Wi-Fi net_if found (index=%d).", net_if_get_by_iface(iface));

    /* Register callbacks for scan results and scan completion. */
    net_mgmt_init_event_callback(
        &wifi_cb,
        wifi_event_handler,
        NET_EVENT_WIFI_SCAN_RESULT |
        NET_EVENT_WIFI_SCAN_DONE);

    net_mgmt_add_event_callback(&wifi_cb);

    wifi_scan(iface);

    LOG_INF("======SCAN COMPLETE======");

    while (1) {
        k_sleep(K_FOREVER);
    }
    return 0;
}
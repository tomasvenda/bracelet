#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h> /* For parsing bands */
#include <zephyr/net/net_event.h>
#include <zephyr/net/ethernet.h>

LOG_MODULE_REGISTER(scan, CONFIG_LOG_DEFAULT_LEVEL);

static uint32_t scan_result_count;
static struct net_mgmt_event_callback wifi_mgmt_cb;
K_SEM_DEFINE(scan_sem, 0, 1);

/* 1. Logic to set MAC address from Kconfig */
static void set_mac_address(struct net_if *iface)
{
    uint8_t mac[6];
    if (gen_mac_addr(mac, sizeof(mac)) == 0) {
        /* If no hardware MAC, use the one from Kconfig */
        if (net_sprint_ll_addr_buf(NULL, 0, NULL, 0)) { // Dummy check
             // Logic to parse CONFIG_WIFI_MAC_ADDRESS string could go here
        }
    }
}

static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
    uint8_t mac_string_buf[sizeof("xx:xx:xx:xx:xx:xx")];
    char ssid_print[WIFI_SSID_MAX_LEN + 1];

    scan_result_count++;
    if (scan_result_count == 1U) {
        printk("\n%-4s | %-32s | %-4s | %-4s | %-5s | %s\n", "Num", "SSID", "Chan", "RSSI", "Sec", "BSSID");
        printk("------------------------------------------------------------------------------\n");
    }

    int ssid_len = MIN(entry->ssid_length, WIFI_SSID_MAX_LEN);
    memcpy(ssid_print, entry->ssid, ssid_len);
    ssid_print[ssid_len] = '\0';

    printk("%-4d | %-32s | %-4u | %-4d | %-5s | %s\n",
           scan_result_count, ssid_print, entry->channel, entry->rssi,
           wifi_security_txt(entry->security),
           net_sprint_ll_addr_buf(entry->mac, WIFI_MAC_ADDR_LEN, mac_string_buf, sizeof(mac_string_buf)));
}

static void handle_wifi_scan_done(struct net_mgmt_event_callback *cb)
{
    k_sem_give(&scan_sem);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) handle_wifi_scan_result(cb);
    if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) handle_wifi_scan_done(cb);
}

static int wifi_scan(void)
{
    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));
    struct wifi_scan_params params = { 0 };

    if (!iface) return -ENOENT;

    /* Use the bands from your Kconfig */
    char bands_list[] = CONFIG_WIFI_SCAN_BANDS_LIST;
    if (strlen(bands_list) > 0) {
        wifi_utils_parse_scan_bands(bands_list, &params.bands);
    }

    /* Set scan type from Kconfig selection */
    params.scan_type = IS_ENABLED(CONFIG_WIFI_SCAN_TYPE_PASSIVE) ? 
                       WIFI_SCAN_TYPE_PASSIVE : WIFI_SCAN_TYPE_ACTIVE;
    
    params.dwell_time_active = CONFIG_WIFI_SCAN_DWELL_TIME_ACTIVE;
    params.dwell_time_passive = CONFIG_WIFI_SCAN_DWELL_TIME_PASSIVE;

    scan_result_count = 0;
    printk("\nScanning (%s)...", (params.scan_type == WIFI_SCAN_TYPE_ACTIVE) ? "Active" : "Passive");

    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(struct wifi_scan_params))) {
        return -ENOEXEC;
    }

    k_sem_take(&scan_sem, K_FOREVER);
    printk(" Done. Found %d networks.\n", scan_result_count);
    return 0;
}

int main(void)
{
    k_sleep(K_SECONDS(1));
    printk("\n*** Thingy:91 X Wi-Fi Scanner Initialized ***\n");

    net_mgmt_init_event_callback(&wifi_mgmt_cb, wifi_mgmt_event_handler, 
                                 NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wifi_mgmt_cb);

    while (1) {
        wifi_scan();
        /* Use the interval from your Kconfig! */
        k_sleep(K_SECONDS(CONFIG_WIFI_SCAN_INTERVAL_S));
    }
    return 0;
}
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h>

#include <string.h>
#include <errno.h>

LOG_MODULE_REGISTER(wifi_app, LOG_LEVEL_DBG);

static uint32_t scan_result_count;
static struct net_mgmt_event_callback wifi_mgmt_cb;
K_SEM_DEFINE(scan_sem, 0, 1);

static void print_iface_cb(struct net_if *iface, void *user_data)
{
	ARG_UNUSED(user_data);

	const struct device *dev = net_if_get_device(iface);
	const char *dev_name = dev ? dev->name : "<no-dev>";
	int index = net_if_get_by_iface(iface);

	printk("iface[%d]: dev=%s up=%d dormant=%d\n",
	       index, dev_name, net_if_is_up(iface), net_if_is_dormant(iface));
}

static bool iface_is_wifi(struct net_if *iface)
{
	struct wifi_iface_status status = {0};
	int ret;

	if (!iface) {
		return false;
	}

	ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status));
	return (ret == 0);
}

static struct net_if *find_wifi_iface(void)
{
	struct net_if *iface = NULL;

	STRUCT_SECTION_FOREACH(net_if, iface) {
		if (iface_is_wifi(iface)) {
			return iface;
		}
	}

	printk("SANITY: No Wi-Fi interface found.\n");
	printk("SANITY: This usually means wlan0 was not created.\n");
	printk("SANITY: Check app.overlay for:\n");
	printk("  - nrf7002 node status = \"okay\"\n");
	printk("  - child node wlan0 { compatible = \"nordic,wlan\"; }\n");
	printk("  - chosen { zephyr,wifi = &wlan0; }\n");

	return NULL;
}

static void dump_wifi_status(struct net_if *iface)
{
	struct wifi_iface_status status = {0};
	int ret;

	if (!iface) {
		printk("SANITY: dump_wifi_status called with NULL iface\n");
		return;
	}

	ret = net_mgmt(NET_REQUEST_WIFI_IFACE_STATUS, iface, &status, sizeof(status));
	if (ret) {
		printk("SANITY: NET_REQUEST_WIFI_IFACE_STATUS failed: %d\n", ret);
		return;
	}

	printk("Wi-Fi status:\n");
	printk("  state=%s\n", wifi_state_txt(status.state));
	printk("  rssi=%d\n", status.rssi);
	printk("  band=%u\n", status.band);
	printk("  channel=%u\n", status.channel);
	printk("  security=%u\n", status.security);
	printk("  iface_mode=%u\n", status.iface_mode);
	printk("  link_mode=%u\n", status.link_mode);
}

static void handle_wifi_scan_result(const struct net_mgmt_event_callback *cb)
{
	const struct wifi_scan_result *entry =
		(const struct wifi_scan_result *)cb->info;
	char ssid_print[WIFI_SSID_MAX_LEN + 1];
	int ssid_len;

	if (!entry) {
		printk("SANITY: scan result callback with NULL entry\n");
		return;
	}

	scan_result_count++;

	if (scan_result_count == 1U) {
		printk("\n%-4s | %-32s | %-4s | %-4s | %-5s | %s\n",
		       "Num", "SSID", "Chan", "RSSI", "Sec", "BSSID");
		printk("------------------------------------------------------------------------------\n");
	}

	ssid_len = MIN(entry->ssid_length, WIFI_SSID_MAX_LEN);
	memcpy(ssid_print, entry->ssid, ssid_len);
	ssid_print[ssid_len] = '\0';

	printk("%-4u | %-32s | %-4u | %-4d | %-5s | %02x:%02x:%02x:%02x:%02x:%02x\n",
	       scan_result_count,
	       ssid_print,
	       entry->channel,
	       entry->rssi,
	       wifi_security_txt(entry->security),
	       entry->mac[0], entry->mac[1], entry->mac[2],
	       entry->mac[3], entry->mac[4], entry->mac[5]);
}

static void handle_wifi_scan_done(const struct net_mgmt_event_callback *cb)
{
	const struct wifi_status *status = (const struct wifi_status *)cb->info;

	if (status) {
		printk("Scan done event: status=%d\n", status->status);
	} else {
		printk("Scan done event with NULL status info\n");
	}

	k_sem_give(&scan_sem);
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb,
				    uint64_t mgmt_event,
				    struct net_if *iface)
{
	printk("MGMT event: 0x%llx on iface=%d\n",
	       mgmt_event,
	       iface ? net_if_get_by_iface(iface) : -1);

	if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) {
		handle_wifi_scan_result(cb);
	} else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) {
		handle_wifi_scan_done(cb);
	}
}

static int wifi_scan(void)
{
	struct net_if *iface = find_wifi_iface();
	struct wifi_scan_params params = {0};
	int ret;
	char bands_list[] = CONFIG_WIFI_SCAN_BANDS_LIST;

	if (!iface) {
		return -ENOENT;
	}

	dump_wifi_status(iface);

	if (strlen(bands_list) > 0U) {
		printk("Scanning bands: %s\n", bands_list);
		ret = wifi_utils_parse_scan_bands(bands_list, &params.bands);
		if (ret) {
			printk("SANITY: wifi_utils_parse_scan_bands failed: %d\n", ret);
			return ret;
		}
	}

	params.scan_type = WIFI_SCAN_TYPE_ACTIVE;
	params.dwell_time_active = CONFIG_WIFI_SCAN_DWELL_TIME_ACTIVE;
	params.dwell_time_passive = CONFIG_WIFI_SCAN_DWELL_TIME_PASSIVE;

	scan_result_count = 0;

	printk("Starting Wi-Fi scan on iface=%d...\n", net_if_get_by_iface(iface));

	ret = net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(params));
	if (ret) {
		printk("ERROR: NET_REQUEST_WIFI_SCAN failed: %d\n", ret);
		printk("SANITY: Common causes:\n");
		printk("  - nRF7002 driver not initialized\n");
		printk("  - wrong devicetree / missing wlan0 child\n");
		printk("  - chip not powered\n");
		printk("  - IRQ pin wrong or not toggling\n");
		return ret;
	}

	ret = k_sem_take(&scan_sem, K_SECONDS(20));
	if (ret) {
		printk("ERROR: Timed out waiting for scan completion: %d\n", ret);
		printk("SANITY: Request was sent but no completion event arrived.\n");
		printk("SANITY: This often points to IRQ / SPI / power problems.\n");
		return ret;
	}

	printk("Scan finished. Found %u networks.\n", scan_result_count);
	return 0;
}

int main(void)
{
	struct net_if *def = net_if_get_default();
	struct net_if *wifi;

	printk("\n*** nRF9151 + nRF7002 Wi-Fi scan sanity app ***\n");

#if DT_NODE_EXISTS(DT_NODELABEL(nrf7002))
	printk("DT sanity: nrf7002 node exists.\n");
#else
	printk("DT sanity: nrf7002 node NOT found.\n");
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(wlan0))
	printk("DT sanity: wlan0 node exists.\n");
#else
	printk("DT sanity: wlan0 node NOT found.\n");
	printk("DT sanity: add wlan0 { compatible = \"nordic,wlan\"; } under nrf7002.\n");
#endif

	printk("Enumerating network interfaces...\n");
	net_if_foreach(print_iface_cb, NULL);

	if (def) {
		const struct device *dev = net_if_get_device(def);
		printk("Default iface=%d dev=%s\n",
		       net_if_get_by_iface(def),
		       dev ? dev->name : "<no-dev>");
	} else {
		printk("SANITY: net_if_get_default() returned NULL\n");
	}

	wifi = find_wifi_iface();
	if (!wifi) {
		printk("FATAL: No Wi-Fi iface available. Retrying every 10s.\n");
	} else {
		printk("Wi-Fi iface found: index=%d\n", net_if_get_by_iface(wifi));
		dump_wifi_status(wifi);
	}

	net_mgmt_init_event_callback(&wifi_mgmt_cb,
				     wifi_mgmt_event_handler,
				     NET_EVENT_WIFI_SCAN_RESULT |
				     NET_EVENT_WIFI_SCAN_DONE);
	net_mgmt_add_event_callback(&wifi_mgmt_cb);

	while (1) {
		int ret = wifi_scan();

		if (ret) {
			printk("wifi_scan() failed: %d\n", ret);
		}

		k_sleep(K_SECONDS(CONFIG_WIFI_SCAN_INTERVAL_S));
	}

	return 0;
}
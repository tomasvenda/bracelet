#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_utils.h>

#include <string.h>
#include <stdio.h>
#include <errno.h>

LOG_MODULE_REGISTER(wifi_app, LOG_LEVEL_DBG);

static uint32_t scan_result_count;
static struct net_mgmt_event_callback wifi_mgmt_cb;
K_SEM_DEFINE(scan_sem, 0, 1);

static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *spi3_dev  = DEVICE_DT_GET(DT_NODELABEL(spi3));

static void format_mac(const uint8_t *mac, char *buf, size_t len)
{
	snprintk(buf, len, "%02X:%02X:%02X:%02X:%02X:%02X",
		 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void wifi_en_selftest(void)
{
	int ret;

	printk("=== WIFI_EN selftest on P0.17 ===\n");

	if (!device_is_ready(gpio0_dev)) {
		printk("GPIO0 not ready\n");
		return;
	}

	ret = gpio_pin_configure(gpio0_dev, 17, GPIO_OUTPUT_INACTIVE);
	printk("WIFI_EN configure P0.17 ret=%d\n", ret);

	ret = gpio_pin_set(gpio0_dev, 17, 0);
	printk("WIFI_EN set low ret=%d\n", ret);
	k_sleep(K_MSEC(20));

	ret = gpio_pin_set(gpio0_dev, 17, 1);
	printk("WIFI_EN set high ret=%d\n", ret);
	k_sleep(K_MSEC(20));

	ret = gpio_pin_set(gpio0_dev, 17, 1);
	printk("WIFI_EN hold high ret=%d\n", ret);
	k_sleep(K_MSEC(20));
}

static void spi_loopback_test(void)
{
	int ret;
	uint8_t tx_buf[] = { 0xAB, 0xCD, 0xEF, 0x42 };
	uint8_t rx_buf[sizeof(tx_buf)] = { 0 };

	struct spi_buf tx = {
		.buf = tx_buf,
		.len = sizeof(tx_buf),
	};

	struct spi_buf rx = {
		.buf = rx_buf,
		.len = sizeof(rx_buf),
	};

	struct spi_buf_set tx_set = {
		.buffers = &tx,
		.count = 1,
	};

	struct spi_buf_set rx_set = {
		.buffers = &rx,
		.count = 1,
	};

	struct spi_config cfg = {
		.frequency = 1000000,
		.operation = SPI_OP_MODE_MASTER |
			     SPI_WORD_SET(8) |
			     SPI_TRANSFER_MSB |
			     SPI_LINES_SINGLE,
		.slave = 0,
		.cs = { 0 },
	};

	printk("=== SPI3 loopback selftest ===\n");
	printk("NOTE: Temporarily short MOSI(P0.14) to MISO(P0.15) for this test.\n");

	if (!device_is_ready(spi3_dev)) {
		printk("SPI3 not ready\n");
		return;
	}

	ret = spi_transceive(spi3_dev, &cfg, &tx_set, &rx_set);

	printk("SPI loopback ret=%d\n", ret);
	printk("TX: %02X %02X %02X %02X\n",
	       tx_buf[0], tx_buf[1], tx_buf[2], tx_buf[3]);
	printk("RX: %02X %02X %02X %02X\n",
	       rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

	if (ret == 0 && memcmp(tx_buf, rx_buf, sizeof(tx_buf)) == 0) {
		printk("SPI loopback PASSED\n");
	} else if (ret == 0) {
		printk("SPI loopback DATA MISMATCH\n");
	} else {
		printk("SPI loopback FAILED\n");
	}
}

static void print_iface_cb(struct net_if *iface, void *user_data)
{
	ARG_UNUSED(user_data);

	const struct device *dev = net_if_get_device(iface);
	const char *dev_name = dev ? dev->name : "<no-dev>";
	int index = net_if_get_by_iface(iface);

	printk("iface[%d]: dev=%s, up=%d, dormant=%d\n",
	       index,
	       dev_name,
	       net_if_is_up(iface),
	       net_if_is_dormant(iface));
}

static struct net_if *find_wifi_iface(void)
{
	struct net_if *iface = net_if_get_default();

	if (!iface) {
		printk("SANITY: net_if_get_default() returned NULL\n");
		return NULL;
	}

	if (strcmp(net_if_get_device(iface)->name, "wlan0") != 0) {
		printk("SANITY: default iface is not wlan0, it is %s\n",
		       net_if_get_device(iface)->name);
	}

	return iface;
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
	const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
	char ssid_print[WIFI_SSID_MAX_LEN + 1];
	char mac_buf[18];
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

	format_mac(entry->mac, mac_buf, sizeof(mac_buf));

	printk("%-4u | %-32s | %-4u | %-4d | %-5s | %s\n",
	       scan_result_count,
	       ssid_print,
	       entry->channel,
	       entry->rssi,
	       wifi_security_txt(entry->security),
	       mac_buf);
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
	ARG_UNUSED(cb);

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

	if (!iface) {
		return -ENOENT;
	}

	dump_wifi_status(iface);

	{
		char bands_list[] = CONFIG_WIFI_SCAN_BANDS_LIST;

		if (strlen(bands_list) > 0) {
			printk("Scanning bands: %s\n", bands_list);
			ret = wifi_utils_parse_scan_bands(bands_list, &params.bands);
			if (ret) {
				printk("SANITY: wifi_utils_parse_scan_bands failed: %d\n", ret);
				return ret;
			}
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
		printk("SANITY: likely causes are nRF7002 init failure, enable pin, SPI, or IRQ.\n");
		return ret;
	}

	ret = k_sem_take(&scan_sem, K_SECONDS(20));
	if (ret) {
		printk("ERROR: Timed out waiting for scan completion: %d\n", ret);
		return ret;
	}

	printk("Scan finished. Found %u networks.\n", scan_result_count);
	return 0;
}

int main(void)
{
	struct net_if *def = net_if_get_default();
	struct net_if *wifi = NULL;

	wifi_en_selftest();
	spi_loopback_test();

	printk("\n*** nRF9151 + nRF7002 Wi-Fi scan sanity app ***\n");

#if DT_NODE_EXISTS(DT_NODELABEL(nrf70))
	printk("DT sanity: nrf70 node exists.\n");
#else
	printk("DT sanity: nrf70 node NOT found.\n");
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(wlan0))
	printk("DT sanity: wlan0 node exists.\n");
#else
	printk("DT sanity: wlan0 node NOT found.\n");
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
		printk("Wi-Fi iface selected: index=%d dev=%s\n",
		       net_if_get_by_iface(wifi),
		       net_if_get_device(wifi)->name);
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
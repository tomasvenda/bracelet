#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_event.h>

LOG_MODULE_REGISTER(wifi_diag, LOG_LEVEL_DBG);

#define HOST_IRQ_PIN 16
#define WIFI_CS_PIN  12

static const struct device *ldo1_dev   = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldsw1));
static const struct device *gpio0_dev  = DEVICE_DT_GET(DT_NODELABEL(gpio0));

/* ---------------------------------------------------------------
 * Scan state
 * ------------------------------------------------------------- */
static struct net_mgmt_event_callback wifi_cb;
static volatile int networks_found = 0;

K_SEM_DEFINE(scan_sem, 0, 1);

/* Forward declaration - must come before check_wifi_scan */
static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t mgmt_event,
                               struct net_if *iface);

/* ---------------------------------------------------------------
 * STEP 1 - Device readiness
 * ------------------------------------------------------------- */
static void check_devices(void)
{
    LOG_INF("--- STEP 1: Device Readiness ---");

    if (!device_is_ready(ldo1_dev)) {
        LOG_ERR("[FAIL] nPM1300 LDO1 not ready. Check I2C: SDA=P0.09 SCL=P0.08");
    } else {
        LOG_INF("[OK]   nPM1300 LDO1 device ready.");
    }

    if (!device_is_ready(gpio0_dev)) {
        LOG_ERR("[FAIL] GPIO0 not ready.");
    } else {
        LOG_INF("[OK]   GPIO0 device ready.");
    }
}

/* ---------------------------------------------------------------
 * STEP 2 - Power rail
 * ------------------------------------------------------------- */
static void check_power(void)
{
    LOG_INF("--- STEP 2: Power Rails ---");

    if (!device_is_ready(ldo1_dev)) {
        LOG_ERR("[SKIP] LDO1 not ready.");
        return;
    }

    if (regulator_is_enabled(ldo1_dev)) {
        LOG_INF("[OK]   LDO1 ON -> nRF7002 VDDMAIN should be 3.3V.");
        LOG_INF("       Multimeter: measure nRF7002 VDDMAIN vs GND, expect ~3.3V.");
    } else {
        LOG_ERR("[FAIL] LDO1 OFF -> nRF7002 has no power. Check PMIC I2C and DT config.");
    }
}

/* ---------------------------------------------------------------
 * STEP 3 - SPI CS idle state
 * ------------------------------------------------------------- */
static void check_spi_cs(void)
{
    LOG_INF("--- STEP 3: SPI CS (P0.12) ---");

    if (!device_is_ready(gpio0_dev)) {
        LOG_ERR("[SKIP] GPIO0 not ready.");
        return;
    }

    gpio_pin_configure(gpio0_dev, WIFI_CS_PIN, GPIO_INPUT);
    int val = gpio_pin_get(gpio0_dev, WIFI_CS_PIN);

    if (val == 1) {
        LOG_INF("[OK]   P0.12 HIGH at idle. Correct (active-low CS).");
        LOG_INF("       Multimeter: measure P0.12 vs GND, expect ~1.8V at idle.");
    } else {
        LOG_ERR("[FAIL] P0.12 LOW at idle. CS should be HIGH when idle.");
        LOG_ERR("       Check for short or bus conflict on P0.12.");
    }
}

/* ---------------------------------------------------------------
 * STEP 4 - Wi-Fi driver and MAC address
 * ------------------------------------------------------------- */
static void check_wifi_driver(void)
{
    LOG_INF("--- STEP 4: Wi-Fi Driver and MAC Address ---");

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));

    if (!iface) {
        LOG_ERR("[FAIL] No Wi-Fi net_if found. Possible causes:");
        LOG_ERR("  1. CONFIG_WIFI_NRF70=y missing");
        LOG_ERR("  2. CONFIG_NET_L2_ETHERNET=y missing");
        LOG_ERR("  3. nRF7002 DT node not okay");
        LOG_ERR("  4. OTP MAC blank - flash wifi_radio_ficr_prog sample");
        LOG_ERR("  5. SPI wiring wrong: SCK=P0.13 MOSI=P0.14 MISO=P0.15 CS=P0.12");
        return;
    }

    LOG_INF("[OK]   Wi-Fi net_if found (index=%d).", net_if_get_by_iface(iface));

    struct net_linkaddr *mac = net_if_get_link_addr(iface);

    if (mac && mac->len >= 6) {
        LOG_INF("[OK]   MAC read over SPI: %02X:%02X:%02X:%02X:%02X:%02X",
                mac->addr[0], mac->addr[1], mac->addr[2],
                mac->addr[3], mac->addr[4], mac->addr[5]);
        LOG_INF("       SPI bus confirmed working. RPU firmware loaded.");

        bool blank = true;
        for (int i = 0; i < 6; i++) {
            if (mac->addr[i] != 0xFF) {
                blank = false;
                break;
            }
        }
        if (blank) {
            LOG_WRN("[WARN] MAC is FF:FF:FF:FF:FF:FF -> OTP not programmed.");
            LOG_WRN("       Use wifi_radio_ficr_prog to write MAC to nRF7002 OTP.");
        }
    } else {
        LOG_WRN("[WARN] Driver loaded but MAC empty. OTP may be blank.");
    }
}

/* ---------------------------------------------------------------
 * STEP 5 - Wi-Fi scan (RF proof of life)
 * ------------------------------------------------------------- */
static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t mgmt_event,
                               struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) {
        const struct wifi_scan_result *entry =
            (const struct wifi_scan_result *)cb->info;
        networks_found++;
        LOG_INF("  [SCAN] #%02d  SSID=%-32s  RSSI=%4d dBm  CH=%d",
                networks_found,
                entry->ssid_length ? (const char *)entry->ssid : "<hidden>",
                entry->rssi,
                entry->channel);

    } else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) {
        const struct wifi_status *status =
            (const struct wifi_status *)cb->info;
        if (status && status->status) {
            LOG_ERR("[FAIL] Scan done with error: %d", status->status);
        } else {
            LOG_INF("[OK]   Scan complete. Networks found: %d", networks_found);
            if (networks_found == 0) {
                LOG_WRN("[WARN] No APs found. Check antenna and 40MHz crystal.");
            }
        }
        k_sem_give(&scan_sem);
    }
}

static void check_wifi_scan(void)
{
    LOG_INF("--- STEP 5: Wi-Fi Scan (RF Proof-of-Life) ---");

    struct net_if *iface = net_if_get_first_by_type(&NET_L2_GET_NAME(ETHERNET));

    if (!iface) {
        LOG_ERR("[SKIP] No Wi-Fi interface. Skipping scan.");
        return;
    }

    networks_found = 0;

    net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler,
                                 NET_EVENT_WIFI_SCAN_RESULT |
                                 NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wifi_cb);

    struct wifi_scan_params params = { 0 };

    LOG_INF("       Triggering scan...");
    int ret = net_mgmt(NET_REQUEST_WIFI_SCAN, iface,
                       &params, sizeof(struct wifi_scan_params));
    if (ret) {
        LOG_ERR("[FAIL] Scan request failed (err=%d).", ret);
        net_mgmt_del_event_callback(&wifi_cb);
        return;
    }

    if (k_sem_take(&scan_sem, K_SECONDS(15)) != 0) {
        LOG_ERR("[FAIL] Scan timed out after 15s.");
        LOG_ERR("       Check: 40MHz crystal, antenna, VDDMAIN=3.3V.");
    }

    net_mgmt_del_event_callback(&wifi_cb);
}

/* ---------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------- */
int main(void)
{
    k_sleep(K_SECONDS(3));

    LOG_INF("==============================================");
    LOG_INF("     nRF7002 HARDWARE DIAGNOSTIC TOOL        ");
    LOG_INF("==============================================");
    LOG_INF("Pins: SCK=P0.13 MOSI=P0.14 MISO=P0.15 CS=P0.12");
    LOG_INF("      BUCKEN (HARDWIRED TO 1.8V) IRQ=P0.16 FlashCS=P0.10");
    LOG_INF("==============================================");

    check_devices();
    k_sleep(K_MSEC(200));

    check_power();
    k_sleep(K_MSEC(200));

    check_spi_cs();
    k_sleep(K_MSEC(200));

    LOG_INF("Waiting 5s for nRF7002 RPU firmware load...");
    k_sleep(K_SECONDS(5));

    check_wifi_driver();
    k_sleep(K_MSEC(500));

    check_wifi_scan();

    LOG_INF("==============================================");
    LOG_INF(" DIAGNOSTIC COMPLETE.");
    LOG_INF("==============================================");

    while (1) {
        k_sleep(K_FOREVER);
    }

    return 0;
}
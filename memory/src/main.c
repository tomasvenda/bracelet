/*
 * W25Q64JW external flash test -- nRF9151 custom bracelet PCB
 * -----------------------------------------------------------
 * The flash shares SPI3 with the nRF7002 Wi-Fi chip:
 *   CS0 = P0.12 (nRF7002),  CS1 = P0.10 (W25Q64JW)
 *
 * Test phases:
 *   1. Device ready check (driver reads + verifies JEDEC ID EF 60 17
 *      at boot -- if this fails, it's wiring/power/CS, not software)
 *   2. Erase / write / read-back verify on one 4 KB sector
 *   3. Pattern sweep + throughput over a 64 KB region
 *   4. BUS SHARING: bring up the nRF7002 and run an active Wi-Fi scan
 *      (init copied from comms.c) while continuously reading + verifying
 *      flash. Any mismatch here = bus contention / CS isolation problem.
 *
 * DESTRUCTIVE: erases the first 64 KB of the flash chip.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/wifi_mgmt.h>
#include <string.h>

LOG_MODULE_REGISTER(flash_test, LOG_LEVEL_INF);

/* ================= FLASH ================= */

#define FLASH_NODE   DT_NODELABEL(w25q64jw)
static const struct device *flash_dev = DEVICE_DT_GET(FLASH_NODE);

#define SECTOR_SIZE  4096
#define PAGE_SIZE    256
#define TEST_OFFSET  0x000000
#define TEST_SPAN    (64 * 1024)   /* 16 sectors */

static uint8_t wr_buf[PAGE_SIZE];
static uint8_t rd_buf[PAGE_SIZE];

/* Deterministic address-derived pattern so any page can be re-verified
 * without storing what was written. */
static void fill_pattern(uint8_t *buf, size_t len, uint32_t addr)
{
    for (size_t i = 0; i < len; i++) {
        uint32_t a = addr + i;
        buf[i] = (uint8_t)((a ^ (a >> 8) ^ 0x5A) + (a >> 16));
    }
}

static int verify_erased(uint32_t offset, size_t len)
{
    for (uint32_t off = offset; off < offset + len; off += PAGE_SIZE) {
        int err = flash_read(flash_dev, off, rd_buf, PAGE_SIZE);
        if (err) {
            LOG_ERR("flash_read @0x%06X failed: %d", off, err);
            return err;
        }
        for (int i = 0; i < PAGE_SIZE; i++) {
            if (rd_buf[i] != 0xFF) {
                LOG_ERR("Not erased @0x%06X: 0x%02X", off + i, rd_buf[i]);
                return -EIO;
            }
        }
    }
    return 0;
}

static int verify_pattern(uint32_t offset, size_t len, int *mismatches)
{
    for (uint32_t off = offset; off < offset + len; off += PAGE_SIZE) {
        int err = flash_read(flash_dev, off, rd_buf, PAGE_SIZE);
        if (err) {
            LOG_ERR("flash_read @0x%06X failed: %d", off, err);
            return err;
        }
        fill_pattern(wr_buf, PAGE_SIZE, off);
        if (memcmp(rd_buf, wr_buf, PAGE_SIZE) != 0) {
            (*mismatches)++;
            for (int i = 0; i < PAGE_SIZE; i++) {
                if (rd_buf[i] != wr_buf[i]) {
                    LOG_ERR("MISMATCH @0x%06X: wrote 0x%02X read 0x%02X",
                            off + i, wr_buf[i], rd_buf[i]);
                    break; /* first byte of this page is enough */
                }
            }
        }
    }
    return 0;
}

/* ============ WI-FI (init copied from comms.c) ============ */

static struct net_mgmt_event_callback wifi_cb;
static K_SEM_DEFINE(scan_done_sem, 0, 1);
static volatile int scan_results;

static void wifi_event_handler(struct net_mgmt_event_callback *cb,
                               uint64_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event) {
    case NET_EVENT_WIFI_SCAN_RESULT:
    {
        const struct wifi_scan_result *entry =
            (const struct wifi_scan_result *)cb->info;
        scan_results++;
        LOG_INF("[%02d] BSSID=%02X:%02X:%02X:%02X:%02X:%02X  RSSI=%d dBm  CH=%d",
                scan_results,
                entry->mac[0], entry->mac[1], entry->mac[2],
                entry->mac[3], entry->mac[4], entry->mac[5],
                entry->rssi, entry->channel);
        break;
    }
    case NET_EVENT_WIFI_SCAN_DONE:
        LOG_INF("Scan done (%d results)", scan_results);
        k_sem_give(&scan_done_sem);
        break;
    default:
        break;
    }
}

/* Same interface bring-up and scan parameters as comms.c do_wifi_scan() */
static int wifi_start_scan(void)
{
    struct net_if *iface = net_if_get_wifi_sta();

    if (!iface) {
        LOG_ERR("No Wi-Fi interface found");
        return -ENODEV;
    }
    LOG_INF("[OK]   Wi-Fi net_if found (index=%d).", net_if_get_by_iface(iface));

    net_if_up(iface);   /* Wakes nRF7002: bucken + iovdd via DT, like comms */

    struct wifi_scan_params params = {0};

    /* Active scan sends probe requests instead of only listening. */
    params.scan_type = WIFI_SCAN_TYPE_ACTIVE;

    /* Scan both 2.4 GHz and 5 GHz. */
    params.bands = BIT(WIFI_FREQ_BAND_2_4_GHZ) |
                   BIT(WIFI_FREQ_BAND_5_GHZ);

    /* Stay longer on each channel. */
    params.dwell_time_active = 120;
    params.dwell_time_passive = 120;

    /* Return as many APs as possible. */
    params.max_bss_cnt = 64;

    scan_results = 0;
    k_sem_reset(&scan_done_sem);

    int ret = net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(params));
    if (ret) {
        LOG_ERR("Failed to start scan (%d)", ret);
        return ret;
    }
    LOG_INF("Scanning...");
    return 0;
}

static void wifi_power_down(void)
{
    struct net_if *iface = net_if_get_wifi_sta();
    if (iface) {
        net_if_down(iface); /* Drops bucken-gpios LOW automatically */
    }
}

/* ================= MAIN ================= */

int main(void)
{
    int err;
    int64_t t0;

    printk("=== W25Q64JW flash + shared-SPI-bus test ===\n");

    /* ---- PHASE 1: device ready (JEDEC ID probe) ---- */
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("Flash device NOT ready.");
        LOG_ERR("The spi-nor driver reads the JEDEC ID at boot and rejects");
        LOG_ERR("the device on mismatch. Check: 1V8_SW present at the chip,");
        LOG_ERR("CS on P0.10, MISO/MOSI not swapped, and that the part is");
        LOG_ERR("really the 1.8V 'JW' (EF 60 17) not the 3.3V 'JV' (EF 40 17).");
        return -1;
    }
    LOG_INF("PHASE 1 PASS: flash ready, JEDEC ID verified (EF 60 17).");

    /* ---- PHASE 2: single-sector erase/write/verify ---- */
    LOG_INF("PHASE 2: erase + write + verify one 4 KB sector @0x%06X", TEST_OFFSET);

    err = flash_erase(flash_dev, TEST_OFFSET, SECTOR_SIZE);
    if (err) {
        LOG_ERR("flash_erase failed: %d", err);
        return -1;
    }
    if (verify_erased(TEST_OFFSET, SECTOR_SIZE) != 0) {
        LOG_ERR("PHASE 2 FAIL: sector did not erase to 0xFF");
        return -1;
    }

    for (uint32_t off = TEST_OFFSET; off < TEST_OFFSET + SECTOR_SIZE; off += PAGE_SIZE) {
        fill_pattern(wr_buf, PAGE_SIZE, off);
        err = flash_write(flash_dev, off, wr_buf, PAGE_SIZE);
        if (err) {
            LOG_ERR("flash_write @0x%06X failed: %d", off, err);
            return -1;
        }
    }
    int mism = 0;
    verify_pattern(TEST_OFFSET, SECTOR_SIZE, &mism);
    if (mism) {
        LOG_ERR("PHASE 2 FAIL: %d page(s) mismatched", mism);
        return -1;
    }
    LOG_INF("PHASE 2 PASS.");

    /* ---- PHASE 3: 64 KB sweep + throughput ---- */
    LOG_INF("PHASE 3: %d KB erase/write/read sweep", TEST_SPAN / 1024);

    t0 = k_uptime_get();
    err = flash_erase(flash_dev, TEST_OFFSET, TEST_SPAN);
    if (err) {
        LOG_ERR("Block erase failed: %d", err);
        return -1;
    }
    LOG_INF("  Erase %d KB: %lld ms", TEST_SPAN / 1024, k_uptime_get() - t0);

    t0 = k_uptime_get();
    for (uint32_t off = TEST_OFFSET; off < TEST_OFFSET + TEST_SPAN; off += PAGE_SIZE) {
        fill_pattern(wr_buf, PAGE_SIZE, off);
        err = flash_write(flash_dev, off, wr_buf, PAGE_SIZE);
        if (err) {
            LOG_ERR("flash_write @0x%06X failed: %d", off, err);
            return -1;
        }
    }
    int64_t dt = k_uptime_get() - t0;
    LOG_INF("  Write %d KB: %lld ms (%lld KB/s)",
            TEST_SPAN / 1024, dt, dt ? (TEST_SPAN / 1024) * 1000 / dt : 0);

    t0 = k_uptime_get();
    mism = 0;
    verify_pattern(TEST_OFFSET, TEST_SPAN, &mism);
    dt = k_uptime_get() - t0;
    LOG_INF("  Read+verify %d KB: %lld ms (%lld KB/s), mismatches: %d",
            TEST_SPAN / 1024, dt, dt ? (TEST_SPAN / 1024) * 1000 / dt : 0, mism);
    if (mism) {
        LOG_ERR("PHASE 3 FAIL.");
        return -1;
    }
    LOG_INF("PHASE 3 PASS.");

    /* ---- PHASE 4: shared-bus stress -- flash reads DURING a Wi-Fi scan ---- */
    LOG_INF("PHASE 4: nRF7002 active scan while hammering flash reads");
    LOG_INF("         (validates SPI3 arbitration + CS isolation)");

    net_mgmt_init_event_callback(&wifi_cb, wifi_event_handler,
                                 NET_EVENT_WIFI_SCAN_RESULT |
                                 NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wifi_cb);

    err = wifi_start_scan();
    if (err) {
        LOG_WRN("Wi-Fi unavailable (%d); skipping the coexistence phase.", err);
    } else {
        int loops = 0;
        mism = 0;
        /* Read + verify the whole 64 KB repeatedly until the scan ends.
         * The Wi-Fi driver is doing SPI traffic to the nRF7002 on the
         * same bus the whole time. Zephyr serializes transactions per
         * controller, so any corruption seen here is HARDWARE (CS
         * glitching, bus loading, drive strength), not scheduling. */
        while (k_sem_take(&scan_done_sem, K_NO_WAIT) != 0) {
            verify_pattern(TEST_OFFSET, TEST_SPAN, &mism);
            loops++;
        }
        LOG_INF("  Verified 64 KB x%d during scan, %d APs seen, mismatches: %d",
                loops, scan_results, mism);
        if (mism) {
            LOG_ERR("PHASE 4 FAIL: bus contention corrupted flash reads!");
        } else {
            LOG_INF("PHASE 4 PASS: flash + Wi-Fi coexist cleanly on SPI3.");
        }
        wifi_power_down();
    }

    printk("=== Flash test complete ===\n");
    return 0;
}
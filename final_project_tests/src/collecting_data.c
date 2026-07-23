/* collecting_data.c -- fall-data logger for Edge Impulse.
 *
 * Build-gated by CONFIG_APP_COLLECT_DATA. Two modes, chosen at boot:
 *
 *   COLLECT (default): label window -> wait for low-g impact -> capture the
 *                      225-sample window (ring + interpolation) -> store to NVS.
 *   DUMP  (press the button during the boot LED sweep): print every stored
 *                      event over the serial console in a logging.py-friendly
 *                      format, then offer a double-press "erase all".
 *
 * No ML / threshold inference here -- pure data collection. Pressure is
 * stored as ABSOLUTE hPa; logging.py converts to delta-from-first offline.
 */
#ifdef CONFIG_APP_COLLECT_DATA

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/regulator.h>
#include <string.h>
#include "events.h"
#include "sensors.h"

LOG_MODULE_REGISTER(collect, LOG_LEVEL_INF);

/* fsm_sub + the channel are DEFINED in fsm.c (compiled in every build; its
 * thread just never starts in test builds). We only observe here. */
ZBUS_OBS_DECLARE(fsm_sub);
ZBUS_CHAN_DECLARE(fsm_events_chan);

#define MAX_RECORDS       100
#define LABEL_WINDOW_MS   5000
#define IMPACT_TIMEOUT_MS 60000
#define NVS_ID_COUNT      1
#define NVS_ID_BASE       100

#if defined(CONFIG_PARTITION_MANAGER_ENABLED)
#include <pm_config.h>
#define STORAGE_FLASH_DEVICE  DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller))
#define STORAGE_OFFSET        PM_NVS_STORAGE_ADDRESS
#define STORAGE_SIZE          PM_NVS_STORAGE_SIZE
#else
#define STORAGE_FLASH_DEVICE  FIXED_PARTITION_DEVICE(storage_partition)
#define STORAGE_OFFSET        FIXED_PARTITION_OFFSET(storage_partition)
#define STORAGE_SIZE          FIXED_PARTITION_SIZE(storage_partition)
#endif

/* 10 B/sample; 4 B header -> 2254 B/event */
struct stored_sample { int16_t x, y, z; int32_t p_milli_hpa; } __packed;
struct stored_event {
    uint8_t  label;        /* 1 = fall, 0 = no_fall               */
    int8_t   cap_ret;      /* sensors_capture_training_window() ret */
    uint16_t n_samples;
    struct stored_sample s[TRAIN_TOTAL_SAMPLES];
} __packed;

static const struct device *ldo1_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldo1));

static struct nvs_fs nvs;
static struct stored_event evt_buf;
static float train_feats[TRAIN_TOTAL_SAMPLES * 4];
static uint16_t rec_count;

static int storage_init(void)
{
    struct flash_pages_info info;
    nvs.flash_device = STORAGE_FLASH_DEVICE;
    nvs.offset = STORAGE_OFFSET;
    if (flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &info)) return -EIO;
    nvs.sector_size = info.size;
    nvs.sector_count = STORAGE_SIZE / info.size;
    if (nvs_mount(&nvs)) return -EIO;
    if (nvs_read(&nvs, NVS_ID_COUNT, &rec_count, sizeof(rec_count)) != sizeof(rec_count))
        rec_count = 0;
    LOG_INF("[COLLECT] Storage: %u KB, %u events stored", STORAGE_SIZE / 1024, rec_count);
    return 0;
}

/* ---- DUMP MODE: print everything, logging.py-compatible ---------------- */
static void dump_mode(void)
{
    struct bracelet_event evt;
    const struct zbus_channel *chan;

    printk("\n===== FALL LOGGER DUMP BEGIN (%u events) =====\n", rec_count);
    for (uint16_t n = 0; n < rec_count; n++) {
        if (nvs_read(&nvs, NVS_ID_BASE + n, &evt_buf, sizeof(evt_buf)) != sizeof(evt_buf)) {
            printk("# ERROR: read event %u failed\n", n);
            continue;
        }
        printk("----- %s_%03u -----\n", evt_buf.label ? "fall" : "no_fall", n);
        printk("# label=%u cap=%d n=%u\n", evt_buf.label, evt_buf.cap_ret, evt_buf.n_samples);
        printk("timestamp_ms,acc_x_g,acc_y_g,acc_z_g,pressure_hpa\n");
        for (int i = 0; i < evt_buf.n_samples; i++) {
            printk("%d,%.2f,%.2f,%.2f,%.3f\n",
                   (i - TRAIN_PAST_SAMPLES) * 20,
                   evt_buf.s[i].x / 16384.0, evt_buf.s[i].y / 16384.0,
                   evt_buf.s[i].z / 16384.0, evt_buf.s[i].p_milli_hpa / 1000.0);
            if ((i % 25) == 24) k_sleep(K_MSEC(5));   /* let the UART drain */
        }
        k_sleep(K_MSEC(20));
    }
    printk("===== FALL LOGGER DUMP END =====\n");
    printk("# Press button to ERASE ALL, or power off.\n");

    while (1) {
        if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_FOREVER)) continue;
        if (evt.type != EVENT_BUTTON_PRESSED) continue;

        printk("# Erase requested. Press again within 3 s to confirm...\n");
        sensors_led_on(LED_RED);
        k_sleep(K_MSEC(500));                                   /* debounce */
        while (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_NO_WAIT) == 0) { } /* drain bounces */

        bool confirmed = false;
        int64_t end = k_uptime_get() + 3000, left;
        while ((left = end - k_uptime_get()) > 0) {
            if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_MSEC(left)) == 0 &&
                evt.type == EVENT_BUTTON_PRESSED) { confirmed = true; break; }
        }
        sensors_led_off(LED_RED);

        if (confirmed) {
            printk("# Erasing...\n");
            nvs_clear(&nvs);
            nvs_mount(&nvs);
            rec_count = 0;
            printk("# Erased. Power cycle to start a new trial.\n");
            sensors_led_on(LED_RED);
            while (1) k_sleep(K_SECONDS(1));   /* halt so we can't re-trigger */
        } else {
            printk("# Erase cancelled. Waiting for new request.\n");
        }
    }
}

/* Returns true if the button was pressed within the window. Clears any stray
 * capture_pending if an impact happens to fire while we're labelling. */
static bool wait_button_window(int64_t ms)
{
    struct bracelet_event evt;
    const struct zbus_channel *chan;
    bool pressed = false;
    int64_t end = k_uptime_get() + ms, left;
    while ((left = end - k_uptime_get()) > 0) {
        if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_MSEC(left))) break;
        if (evt.type == EVENT_BUTTON_PRESSED)     pressed = true;
        if (evt.type == EVENT_IMU_HARSH_IMPACT)   sensors_evaluation_done();
    }
    return pressed;
}

void collect_data_run(void)
{
    struct bracelet_event evt;
    const struct zbus_channel *chan;

    sensors_disable_motion_trigger();          /* only low-g triggers matter here */
    if (storage_init()) { LOG_ERR("[COLLECT] Storage failed."); return; }

    if (device_is_ready(ldo1_dev)) {
        regulator_enable(ldo1_dev);
        k_sleep(K_MSEC(100));                  /* let the LED rail settle */
    } else {
        LOG_ERR("[COLLECT] LDO1 not ready -- LEDs will stay dark.");
    }

    /* Boot fork: RGB sweep; a press during it -> DUMP mode (never returns). */
    sensors_led_on(LED_RED);   k_sleep(K_MSEC(300)); sensors_led_off(LED_RED);
    sensors_led_on(LED_GREEN); k_sleep(K_MSEC(300)); sensors_led_off(LED_GREEN);
    sensors_led_on(LED_BLUE);  k_sleep(K_MSEC(300)); sensors_led_off(LED_BLUE);
    if (wait_button_window(2100)) dump_mode();

    LOG_INF("[COLLECT] Trial mode, resuming at %u/%u.", rec_count, MAX_RECORDS);

    while (rec_count < MAX_RECORDS) {
        memset(&evt_buf, 0, sizeof(evt_buf));

        /* 1. LABEL WINDOW -- solid BLUE. Press = fall, timeout = no_fall. */
        sensors_led_on(LED_BLUE);
        evt_buf.label = wait_button_window(LABEL_WINDOW_MS) ? 1 : 0;
        sensors_led_off(LED_BLUE);
        if (evt_buf.label) for (int i = 0; i < 2; i++) {   /* 2 blinks = "fall armed" */
            k_sleep(K_MSEC(150)); sensors_led_on(LED_BLUE);
            k_sleep(K_MSEC(150)); sensors_led_off(LED_BLUE);
        }

        /* 2. WAIT FOR IMPACT (low-g) -- LEDs off to save power over 60 s. */
        bool got = false;
        int64_t end = k_uptime_get() + IMPACT_TIMEOUT_MS, left;
        while ((left = end - k_uptime_get()) > 0) {
            if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_MSEC(left))) break;
            if (evt.type == EVENT_IMU_HARSH_IMPACT) { got = true; break; }
        }
        if (!got) continue;                    /* timed out -> re-label, nothing stored */

        /* 3. CAPTURE (ring + interpolation, ~4.5 s) -- BLUE = capturing. */
        sensors_led_on(LED_BLUE);
        evt_buf.cap_ret = sensors_capture_training_window(train_feats,
                                                          ARRAY_SIZE(train_feats));
        sensors_led_off(LED_BLUE);

        if (evt_buf.cap_ret == 0) {
            evt_buf.n_samples = TRAIN_TOTAL_SAMPLES;
            for (int i = 0; i < TRAIN_TOTAL_SAMPLES; i++) {
                evt_buf.s[i].x = (int16_t)(train_feats[i*4+0] * 16384.0f);
                evt_buf.s[i].y = (int16_t)(train_feats[i*4+1] * 16384.0f);
                evt_buf.s[i].z = (int16_t)(train_feats[i*4+2] * 16384.0f);
                evt_buf.s[i].p_milli_hpa = (int32_t)(train_feats[i*4+3] * 1000.0f);
            }
        }
        sensors_evaluation_done();             /* clears capture_pending -> ring resumes */

        /* 4. STORE + feedback. Capture failed -> 3 red blinks, not stored. */
        if (evt_buf.cap_ret != 0) {
            LOG_WRN("[COLLECT] Capture failed (%d) -- not stored.", evt_buf.cap_ret);
            for (int i = 0; i < 3; i++) {
                sensors_led_on(LED_RED);  k_sleep(K_MSEC(80));
                sensors_led_off(LED_RED); k_sleep(K_MSEC(80));
            }
            continue;
        }

        nvs_write(&nvs, NVS_ID_BASE + rec_count, &evt_buf, sizeof(evt_buf));
        rec_count++;
        nvs_write(&nvs, NVS_ID_COUNT, &rec_count, sizeof(rec_count));
        LOG_INF("[COLLECT] Stored %s event %u.",
                evt_buf.label ? "FALL" : "no_fall", rec_count - 1);

        /* Verdict LED = the label: RED = fall, GREEN = no_fall. */
        sensors_led_on(evt_buf.label ? LED_RED : LED_GREEN);
        k_sleep(K_SECONDS(2));
        sensors_led_off(LED_RED); sensors_led_off(LED_GREEN);

        struct bracelet_event stale;
        const struct zbus_channel *stale_chan;
        while (zbus_sub_wait_msg(&fsm_sub, &stale_chan, &stale, K_NO_WAIT) == 0) {
            /* discard */
        }
        sensors_evaluation_done();
    }

    LOG_INF("[COLLECT] %u records stored. Trial full -- dump & erase.", MAX_RECORDS);
    while (1) {                                 /* idle heartbeat */
        for (int i = 0; i < 3; i++) {
            sensors_led_on(LED_GREEN);  k_sleep(K_MSEC(120));
            sensors_led_off(LED_GREEN); k_sleep(K_MSEC(120));
        }
        k_sleep(K_SECONDS(2));
    }
}

#endif /* CONFIG_APP_COLLECT_DATA */
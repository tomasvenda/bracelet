#ifdef CONFIG_APP_DETECTOR_ACCURACY_TEST

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/regulator.h>
#include "events.h"
#include "sensors.h"
#include "ml_wrapper.h"
#include "threshold_model.h"

LOG_MODULE_REGISTER(det_acc, LOG_LEVEL_INF);

ZBUS_OBS_DECLARE(fsm_sub);
ZBUS_CHAN_DECLARE(fsm_events_chan);
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

struct stored_sample { int16_t x, y, z; int32_t p_milli_hpa; } __packed;   /* 10 B */
struct stored_event {
    uint8_t  label;                 /* 1 = fall */
    int8_t   ml, th, cap_ret;
    float    ml_conf, ff_g, imp_g, dp_hpa, ml_anom;
    uint16_t n_samples, reserved;
    struct stored_sample s[TRAIN_TOTAL_SAMPLES];
} __packed;                          /* 24 + 2250 = 2274 B */


static const struct device *ldo1_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldo1));

static struct nvs_fs nvs;
static struct stored_event evt_buf;
static float train_feats[TRAIN_TOTAL_SAMPLES * 4];
static float old_feats[150 * 4];
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
    LOG_INF("[DA] Storage: %u KB, %u events stored", STORAGE_SIZE / 1024, rec_count);
    return 0;
}

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
        printk("# ml=%d th=%d conf=%.3f anom=%.3f ff=%.2f imp=%.2f dp=%.3f cap=%d\n",
               evt_buf.ml, evt_buf.th, (double)evt_buf.ml_conf, (double)evt_buf.ml_anom,
               (double)evt_buf.ff_g, (double)evt_buf.imp_g, (double)evt_buf.dp_hpa, evt_buf.cap_ret);
        printk("timestamp_ms,acc_x_g,acc_y_g,acc_z_g,pressure_hpa\n");
        for (int i = 0; i < evt_buf.n_samples; i++) {
            printk("%d,%.2f,%.2f,%.2f,%.3f\n",
                   (i - TRAIN_PAST_SAMPLES) * 20,
                   evt_buf.s[i].x / 16384.0, evt_buf.s[i].y / 16384.0,
                   evt_buf.s[i].z / 16384.0, evt_buf.s[i].p_milli_hpa / 1000.0);
            if ((i % 25) == 24) k_sleep(K_MSEC(5));
        }
        k_sleep(K_MSEC(20));
    }
    printk("===== FALL LOGGER DUMP END =====\n");
    printk("# Press button to ERASE ALL, or power off.\n");

    while (1) {
        /* 1. Wait for the initial button press */
        if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_FOREVER)) continue;
        if (evt.type != EVENT_BUTTON_PRESSED) continue;

        printk("# Erase requested. Press again within 3 seconds to confirm...\n");
        sensors_led_on(LED_RED);

        /* 2. Software debounce: wait 500ms to ignore accidental bounces from the first press */
        k_sleep(K_MSEC(500));

        /* 3. Drain any lingering bounce messages sitting in the zbus queue */
        while (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_NO_WAIT) == 0) {
            /* do nothing, just empty the queue */
        }

        /* 4. Wait for the deliberate second press (3-second timeout) */
        bool confirmed = false;
        int64_t end_time = k_uptime_get() + 3000;
        int64_t left;
        
        while ((left = end_time - k_uptime_get()) > 0) {
            if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_MSEC(left)) == 0) {
                if (evt.type == EVENT_BUTTON_PRESSED) {
                    confirmed = true;
                    break;
                }
            }
        }

        sensors_led_off(LED_RED);

        /* 5. Execute or Cancel */
        if (confirmed) {
            printk("# Erasing...\n");
            nvs_clear(&nvs); 
            nvs_mount(&nvs);
            rec_count = 0;
            printk("# Erased. Power cycle to start a new trial.\n");
            
            /* Turn LED solid RED and halt the system so it doesn't accidentally trigger again */
            sensors_led_on(LED_RED);
            while(1) { 
                k_sleep(K_SECONDS(1)); 
            }
        } else {
            printk("# Erase cancelled. Waiting for new request.\n");
        }
    }
}


static bool wait_button_window(int64_t ms)
{
    struct bracelet_event evt;
    const struct zbus_channel *chan;
    bool pressed = false;
    int64_t end = k_uptime_get() + ms, left;
    while ((left = end - k_uptime_get()) > 0) {
        if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_MSEC(left))) break;
        if (evt.type == EVENT_BUTTON_PRESSED) pressed = true;
        if (evt.type == EVENT_IMU_HARSH_IMPACT) sensors_evaluation_done();
    }
    return pressed;
}

void detector_accuracy_test_run(void)
{
    struct bracelet_event evt;
    const struct zbus_channel *chan;

    sensors_disable_motion_trigger();
    if (storage_init()) { LOG_ERR("[DA] Storage failed."); return; }

    if (device_is_ready(ldo1_dev)) {
        regulator_enable(ldo1_dev);
        k_sleep(K_MSEC(100));   /* let the buck-boost rail settle */
    } else {
        LOG_ERR("[DA] LDO1 not ready -- LEDs will stay dark.");
    }

    /* Boot fork: LED sweep, press button -> dump mode */
    sensors_led_on(LED_RED);   k_sleep(K_MSEC(300)); sensors_led_off(LED_RED);
    sensors_led_on(LED_GREEN); k_sleep(K_MSEC(300)); sensors_led_off(LED_GREEN);
    sensors_led_on(LED_BLUE);  k_sleep(K_MSEC(300)); sensors_led_off(LED_BLUE);
    if (wait_button_window(2100)) dump_mode();          /* never returns */

    LOG_INF("[DA] Trial mode, resuming at %u/%u.", rec_count, MAX_RECORDS);

    while (rec_count < MAX_RECORDS) {
        memset(&evt_buf, 0, sizeof(evt_buf));

        /* 1. Label window: BLUE on, button = fall */
        sensors_led_on(LED_BLUE);
        evt_buf.label = wait_button_window(LABEL_WINDOW_MS) ? 1 : 0;
        sensors_led_off(LED_BLUE);
        if (evt_buf.label) for (int i = 0; i < 2; i++) {
            k_sleep(K_MSEC(150)); sensors_led_on(LED_BLUE);
            k_sleep(K_MSEC(150)); sensors_led_off(LED_BLUE);
        }

        /* 2. Wait for impact */
        bool got = false;
        int64_t end = k_uptime_get() + IMPACT_TIMEOUT_MS, left;
        while ((left = end - k_uptime_get()) > 0) {
            if (zbus_sub_wait_msg(&fsm_sub, &chan, &evt, K_MSEC(left))) break;
            if (evt.type == EVENT_IMU_HARSH_IMPACT) { got = true; break; }
        }
        if (!got) continue;

        /* 3. Extended capture (4.5 s) */
        evt_buf.cap_ret = sensors_capture_training_window(train_feats,
                                                          ARRAY_SIZE(train_feats));
        if (evt_buf.cap_ret == 0) {
            /* Old model verdict: frames 0..149, pressure as delta-from-first */
            float p0 = train_feats[3];
            for (int i = 0; i < 150; i++) {
                old_feats[i*4+0] = train_feats[i*4+0];
                old_feats[i*4+1] = train_feats[i*4+1];
                old_feats[i*4+2] = train_feats[i*4+2];
                old_feats[i*4+3] = train_feats[i*4+3] - p0;
            }
            /* Use local floats to avoid packed struct pointer misalignment */
            float ff, imp, dp;
            
            evt_buf.ml = run_fall_inference(old_feats, ARRAY_SIZE(old_feats));
            evt_buf.ml_conf = ml_last_confidence();
            evt_buf.ml_anom = ml_last_anomaly();
            
            evt_buf.th = run_threshold_inference(old_feats, ARRAY_SIZE(old_feats), &ff, &imp, &dp);
            
            /* Safely assign the local floats back into the packed struct */
            evt_buf.ff_g = ff;
            evt_buf.imp_g = imp;
            evt_buf.dp_hpa = dp;

            evt_buf.n_samples = TRAIN_TOTAL_SAMPLES;
            for (int i = 0; i < TRAIN_TOTAL_SAMPLES; i++) {
                evt_buf.s[i].x = (int16_t)(train_feats[i*4+0] * 16384.0f);
                evt_buf.s[i].y = (int16_t)(train_feats[i*4+1] * 16384.0f);
                evt_buf.s[i].z = (int16_t)(train_feats[i*4+2] * 16384.0f);
                evt_buf.s[i].p_milli_hpa = (int32_t)(train_feats[i*4+3] * 1000.0f);
            }
        } else {
            evt_buf.ml = evt_buf.th = -1;
        }
        sensors_evaluation_done();

        /* 4. Store + verdict LED (RED both fall, GREEN both no, YELLOW disagree) */
        nvs_write(&nvs, NVS_ID_BASE + rec_count, &evt_buf, sizeof(evt_buf));
        rec_count++;
        nvs_write(&nvs, NVS_ID_COUNT, &rec_count, sizeof(rec_count));

        if (evt_buf.ml == 1 && evt_buf.th == 1)      sensors_led_on(LED_RED);
        else if (evt_buf.ml == 0 && evt_buf.th == 0) sensors_led_on(LED_GREEN);
        else { sensors_led_on(LED_RED); sensors_led_on(LED_GREEN); }
        k_sleep(K_SECONDS(2));
        sensors_led_off(LED_RED); sensors_led_off(LED_GREEN);
    }

    LOG_INF("[DA] %u records stored. Done.", MAX_RECORDS);
    while (1) {
        for (int i = 0; i < 3; i++) {
            sensors_led_on(LED_GREEN); k_sleep(K_MSEC(120));
            sensors_led_off(LED_GREEN); k_sleep(K_MSEC(120));
        }
        k_sleep(K_SECONDS(2));
    }
}

#endif
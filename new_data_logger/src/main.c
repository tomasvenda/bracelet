/* =====================================================================
 * Fall Logger -- OFFLINE FLASH LOGGING MODE
 *
 * LTE/MQTT is disabled (all original code preserved inside #if 0
 * blocks, ready to re-enable once the battery is attached).
 *
 * Events are now stored in the nRF9151's internal flash using
 * Zephyr NVS on the 'storage_partition'. Each event is packed to
 * 1512 bytes (int16 accel counts + int32 ABSOLUTE pressure in
 * centi-hPa), so the default 24KB partition holds ~12 events.
 *
 * NEW BUTTON BEHAVIOR (device connected to USB console):
 *   - Short press          -> dump all stored events as JSON over UART
 *   - Hold >= 2 seconds    -> erase all stored events
 *
 * LEDs:
 *   - Green solid  = armed, ready for a drop
 *   - Green off    = busy capturing/storing
 *   - Blue solid   = dump in progress
 *   - Red solid    = storage full (events are being discarded)
 * ===================================================================== */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* --- Modem & MQTT Headers: DISABLED (offline flash logging) --- */
#if 0
#include <nrf_modem_at.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <net/mqtt_helper.h>
#endif

#include "bmi2_defs.h"
#include "bmi270_legacy.h"

LOG_MODULE_REGISTER(fall_logger, LOG_LEVEL_INF);

/* =====================================================================
 * HARDWARE CONFIGURATION
 * ===================================================================== */
#define BMI270_NODE   DT_NODELABEL(bmi270)
#define ICP20100_NODE DT_NODELABEL(icp20100)
#define LDO2_NODE     DT_NODELABEL(npm1300_ldsw2)

/* BMI270 Registers */
#define BMI270_REG_CMD              0x7E
#define BMI270_CMD_FIFO_FLUSH       0xB0
#define BMI270_REG_FIFO_CONFIG_0    0x48
#define BMI270_REG_FIFO_CONFIG_1    0x49
#define BMI270_REG_FIFO_LENGTH_0    0x24
#define BMI270_REG_FIFO_DATA        0x26

/* ICP-20100 Registers */
#define ICP20100_REG_MODE_SELECT 0xC0
#define ICP20100_REG_FIFO_FILL   0xC4
#define ICP20100_REG_FIFO_BASE   0xFA
#define ICP20100_REG_DUMMY       0x00
#define ICP20100_FIFO_LEVEL_MASK 0x1F
#define ICP20100_CMD_FIFO_FLUSH  0x80

#define HALF_WINDOW_BYTES   450
#define HALF_WINDOW_SAMPLES 75
#define FULL_WINDOW_SAMPLES (HALF_WINDOW_SAMPLES * 2) /* 150 samples = 3s total */

const struct i2c_dt_spec bmi_i2c = I2C_DT_SPEC_GET(BMI270_NODE);
const struct i2c_dt_spec icp_i2c = I2C_DT_SPEC_GET(ICP20100_NODE);
static const struct gpio_dt_spec bmi_int = GPIO_DT_SPEC_GET(BMI270_NODE, irq_gpios);
static struct gpio_callback bmi_int_cb;
K_SEM_DEFINE(bmi_irq_sem, 0, 1);

/* --- User button (P0.06) & RGB LED (P0.28/29/30) --- */
static const struct gpio_dt_spec user_button = GPIO_DT_SPEC_GET(DT_NODELABEL(button0), gpios);
static const struct gpio_dt_spec led_red     = GPIO_DT_SPEC_GET(DT_NODELABEL(led_red), gpios);
static const struct gpio_dt_spec led_green   = GPIO_DT_SPEC_GET(DT_NODELABEL(led_green), gpios);
static const struct gpio_dt_spec led_blue    = GPIO_DT_SPEC_GET(DT_NODELABEL(led_blue), gpios);
static struct gpio_callback button_cb;
K_SEM_DEFINE(button_sem, 0, 1);

static uint8_t bmi_fifo_buffer[HALF_WINDOW_BYTES];
static uint8_t icp_fifo_buffer[16 * 6];

/* =====================================================================
 * MQTT CONFIGURATION -- DISABLED (offline flash logging)
 * ===================================================================== */
#if 0
#define MQTT_PUB_TOPIC "bracelet/prototype_pcb/data/training"
#define CLIENT_ID "prototype_pcb"
#define MQTT_BROKER_HOSTNAME "20.251.201.46"

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(mqtt_connected_sem, 0, 1);
static bool mqtt_is_connected = false;
static bool network_active = true;   /* false after graceful button shutdown */
#endif

struct sensor_record {
    int64_t timestamp; /* relative to impact (t=0) */
    double x, y, z;
    double p_hpa;
    bool press_valid;
};

static struct sensor_record event_payload[FULL_WINDOW_SAMPLES];

/* =====================================================================
 * FLASH STORAGE (NVS on internal flash)
 *
 * Layout:
 *   NVS id 1                 -> uint16_t stored event count
 *   NVS id 100 + n           -> packed event record n (0-based)
 *
/* Each sample is packed to 10 bytes:
 *   x/y/z as raw int16 accel counts (value_g = raw / 16384.0)
 *   p as int32 centi-hPa ABSOLUTE   (value_hpa = raw / 100.0)
 * Timestamps are not stored -- they are reconstructed on dump as
 * t = (index - 75) * 20 ms, exactly as they were generated.
 * ===================================================================== */
/* Partition geometry: when the NCS Partition Manager governs the
 * flash layout (this build -- see build/partitions.yml), the true
 * source of the nvs_storage location is pm_config.h, NOT devicetree.
 * The DTS storage_partition label only coincidentally matched before
 * the resize; trusting it after moving the partition would mount NVS
 * at a stale address. Fall back to DTS macros for non-PM builds. */
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

#define NVS_ID_EVENT_COUNT    1
#define NVS_ID_EVENT_BASE     100
#define MAX_STORED_EVENTS     500  /* NVS id cap; flash capacity fills first */

struct stored_sample {
    int16_t x_raw;
    int16_t y_raw;
    int16_t z_raw;
    int32_t p_centi_hpa;   /* ABSOLUTE pressure, hPa * 100 */
} __packed;

struct stored_event {
    int64_t  session_id;
    uint16_t n_samples;
    uint16_t reserved;
    struct stored_sample s[FULL_WINDOW_SAMPLES];
} __packed;   /* 12 + 150*10 = 1512 bytes */

static struct nvs_fs nvs;
static struct stored_event evt_buf;   /* static: too big for the stack */
static uint16_t stored_count;
static bool storage_ok = false;
static bool storage_full = false;

/* =====================================================================
 * LED HELPERS
 * ===================================================================== */
static void led_ready_set(bool on)
{
    /* Green = armed & ready for a new measurement */
    gpio_pin_set_dt(&led_green, on ? 1 : 0);
}

static void led_dump_set(bool on)
{
    /* Blue = dump over UART in progress */
    gpio_pin_set_dt(&led_blue, on ? 1 : 0);
}

static void led_full_set(bool on)
{
    /* Red = storage full, new events are discarded */
    gpio_pin_set_dt(&led_red, on ? 1 : 0);
}

/* =====================================================================
 * BOOT SEQUENCE
 * ===================================================================== */
static int power_up_imu_during_boot(void)
{
    const struct device *const ldo2_dev = DEVICE_DT_GET(LDO2_NODE);
    if (device_is_ready(ldo2_dev)) {
        regulator_enable(ldo2_dev);
    }
    k_sleep(K_MSEC(100));
    return 0;
}
SYS_INIT(power_up_imu_during_boot, POST_KERNEL, 85);

void bmi_isr_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    k_sem_give(&bmi_irq_sem);
}

static void button_isr_handler(const struct device *port, struct gpio_callback *cb, gpio_port_pins_t pins)
{
    /* Simple time-based debounce (ignore bounces < 300 ms apart) */
    static int64_t last_press;
    int64_t now = k_uptime_get();

    if ((now - last_press) < 300) {
        return;
    }
    last_press = now;
    k_sem_give(&button_sem);
}

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

/* =====================================================================
 * NETWORK & MQTT CALLBACKS -- DISABLED (offline flash logging)
 * ===================================================================== */
#if 0
static void lte_handler(const struct lte_lc_evt *const evt)
{
    if (evt->type == LTE_LC_EVT_NW_REG_STATUS) {
        if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
            (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            LOG_INF("Connected to LTE network!");
            k_sem_give(&lte_connected);
        }
    }
}

static void on_mqtt_connack(enum mqtt_conn_return_code return_code, bool session_present)
{
    if (return_code == MQTT_CONNECTION_ACCEPTED) {
        LOG_INF("Connected to MQTT broker!");
        mqtt_is_connected = true;
        k_sem_give(&mqtt_connected_sem);
    } else {
        LOG_WRN("MQTT connection failed, code: %d", return_code);
    }
}

static void on_mqtt_disconnect(int result)
{
    LOG_INF("MQTT client disconnected: %d", result);
    mqtt_is_connected = false;
}
#endif

/* =====================================================================
 * SENSOR DRIVERS
 * ===================================================================== */
static int configure_bmi_fifo(void)
{
    int ret;
    uint8_t fifo_cfg, fifo0;
    i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, &fifo0);
    fifo0 &= ~(1 << 0);
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_0, fifo0);

    ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, &fifo_cfg);
    if (ret) return ret;
    fifo_cfg |= (1 << 6);   /* ACC_EN = 1 */
    fifo_cfg &= ~(1 << 5);  /* GYR_EN = 0 */
    fifo_cfg &= ~(1 << 4);  /* HEADER_EN = 0 */
    ret = i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, fifo_cfg);
    if (ret) return ret;

    return i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
}

static uint16_t read_bmi_half_window(struct sensor_record *out_buf,
                                     uint16_t write_offset,
                                     const char *label)
{
    uint8_t len_buf[2];
    int ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_LENGTH_0, len_buf, 2);
    if (ret) {
        LOG_ERR("[%s] FIFO length read failed: %d", label, ret);
        return 0;
    }

    uint16_t fifo_len = len_buf[0] | ((len_buf[1] & 0x1F) << 8);
    uint16_t total_bytes  = (fifo_len / 6) * 6;

    LOG_INF("[%s] FIFO holds %d bytes (%d frames)",
            label, fifo_len, total_bytes / 6);

    if (total_bytes == 0) {
        LOG_WRN("[%s] FIFO empty.", label);
        return 0;
    }

    if (total_bytes > HALF_WINDOW_BYTES) {
        uint16_t discard = total_bytes - HALF_WINDOW_BYTES;
        LOG_INF("[%s] Discarding %d stale bytes", label, discard);
        uint8_t trash[64];
        while (discard > 0) {
            uint16_t chunk = (discard > sizeof(trash)) ? sizeof(trash) : discard;
            ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, trash, chunk);
            if (ret) return 0;
            discard -= chunk;
        }
    }

    uint16_t keep_bytes = (total_bytes > HALF_WINDOW_BYTES) ? HALF_WINDOW_BYTES : total_bytes;
    ret = i2c_burst_read_dt(&bmi_i2c, BMI270_REG_FIFO_DATA, bmi_fifo_buffer, keep_bytes);
    if (ret) return 0;

    uint16_t sample_count = keep_bytes / 6;

    /* Safety clamp: never write past the payload buffer */
    if ((write_offset + sample_count) > FULL_WINDOW_SAMPLES) {
        sample_count = FULL_WINDOW_SAMPLES - write_offset;
    }

    for (uint16_t i = 0; i < sample_count; i++) {
        int16_t raw_x = (int16_t)((bmi_fifo_buffer[i*6 + 1] << 8) | bmi_fifo_buffer[i*6 + 0]);
        int16_t raw_y = (int16_t)((bmi_fifo_buffer[i*6 + 3] << 8) | bmi_fifo_buffer[i*6 + 2]);
        int16_t raw_z = (int16_t)((bmi_fifo_buffer[i*6 + 5] << 8) | bmi_fifo_buffer[i*6 + 4]);

        out_buf[write_offset + i].x = raw_x / 16384.0;
        out_buf[write_offset + i].y = raw_y / 16384.0;
        out_buf[write_offset + i].z = raw_z / 16384.0;
        out_buf[write_offset + i].timestamp = ((int64_t)(write_offset + i) - HALF_WINDOW_SAMPLES) * 20;
    }

    LOG_INF("[%s] Wrote %d samples (indices %d..%d)",
            label, sample_count,
            write_offset, write_offset + sample_count - 1);

    /* CRASH FIX: only hexdump when we actually have >= 16 bytes.
     * Previously &bmi_fifo_buffer[keep_bytes - 16] underflowed on short
     * reads (re-trigger right after a FIFO flush) -> bus fault.
     */
    if ((strcmp(label, "FUTURE") == 0) && (keep_bytes >= 16)) {
        LOG_HEXDUMP_INF(bmi_fifo_buffer, 16, "First 16 raw bytes:");
        LOG_HEXDUMP_INF(&bmi_fifo_buffer[keep_bytes - 16], 16, "Last 16 raw bytes:");
    }
    return sample_count;
}

static void flush_pressure_fifo(void)
{
    uint8_t current_fill = 0;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &current_fill);
    current_fill |= ICP20100_CMD_FIFO_FLUSH;
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, current_fill);
}

static uint16_t read_pressure_fifo(float *out_pressure, uint16_t max_samples)
{
    uint8_t fifo_fill_reg = 0;
    int ret = i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_FIFO_FILL, &fifo_fill_reg);
    if (ret) return 0;

    uint8_t fifo_count = fifo_fill_reg & ICP20100_FIFO_LEVEL_MASK;
    if (fifo_count == 0) return 0;
    if (fifo_count > max_samples) fifo_count = max_samples;

    uint16_t bytes_to_read = fifo_count * 6;
    ret = i2c_burst_read_dt(&icp_i2c, ICP20100_REG_FIFO_BASE, icp_fifo_buffer, bytes_to_read);
    if (ret) return 0;

    uint8_t dummy;
    i2c_reg_read_byte_dt(&icp_i2c, ICP20100_REG_DUMMY, &dummy);

    for (int i = 0; i < fifo_count; i++) {
        uint8_t *packet = &icp_fifo_buffer[i * 6];
        int32_t data_press = ((int32_t)(packet[2] & 0x0f) << 16) |
                             ((int32_t)packet[1] << 8) |
                             packet[0];

        if (data_press & 0x080000) data_press |= 0xFFF00000;

        out_pressure[i] = ((float)(data_press) * 40.0f / 131072.0f) + 70.0f;
    }
    return fifo_count;
}

/* =====================================================================
 * DATA PROCESSING
 * ===================================================================== */
static void align_and_pad_pressure(float *past_p, uint16_t past_n, float *future_p, uint16_t future_n)
{
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        event_payload[i].press_valid = false;
    }

    /* Map past pressure (ending at t=0, index 74) spaced by 2 indices (25Hz) */
    int idx = HALF_WINDOW_SAMPLES - 1 - ((past_n - 1) * 2);
    if (idx < 0) idx = 0;
    for (int i = 0; i < past_n; i++) {
        if (idx >= HALF_WINDOW_SAMPLES) break;
        event_payload[idx].p_hpa = past_p[i] * 10.0f; /* Convert kPa to hPa */
        event_payload[idx].press_valid = true;
        idx += 2;
    }

    /* Map future pressure (ending at t=1.5s, index 149) */
    idx = FULL_WINDOW_SAMPLES - 1 - ((future_n - 1) * 2);
    if (idx < HALF_WINDOW_SAMPLES) idx = HALF_WINDOW_SAMPLES;
    for (int i = 0; i < future_n; i++) {
        if (idx >= FULL_WINDOW_SAMPLES) break;
        event_payload[idx].p_hpa = future_p[i] * 10.0f;
        event_payload[idx].press_valid = true;
        idx += 2;
    }

    /* Forward Fill Padding */
    double last_p = past_n > 0 ? ((double)past_p[0] * 10.0) : 1013.25;
    bool has_p = false;
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (event_payload[i].press_valid) {
            last_p = event_payload[i].p_hpa;
            has_p = true;
        } else if (has_p) {
            event_payload[i].p_hpa = last_p;
            event_payload[i].press_valid = true;
        }
    }

    /* Backward Fill Padding (for the very beginning if empty) */
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (event_payload[i].press_valid) {
            last_p = event_payload[i].p_hpa;
            break;
        }
    }
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        if (!event_payload[i].press_valid) {
            event_payload[i].p_hpa = last_p;
            event_payload[i].press_valid = true;
        } else {
            break;
        }
    }

    /* BASELINE SUBTRACTION DISABLED for debugging: p_hpa now stays
     * ABSOLUTE (should read ~1013 hPa near sea level). Re-enable this
     * block to go back to delta-pressure once the sensor pipeline is
     * verified. */
#if 0
    double baseline_p = event_payload[0].p_hpa;
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        event_payload[i].p_hpa = event_payload[i].p_hpa - baseline_p;
    }
#endif
}

/* =====================================================================
 * FLASH STORAGE: init / store / dump / erase
 * ===================================================================== */
static int init_storage(void)
{
    int rc;
    struct flash_pages_info page_info;

    nvs.flash_device = STORAGE_FLASH_DEVICE;
    if (!device_is_ready(nvs.flash_device)) {
        LOG_ERR("Flash device not ready");
        return -ENODEV;
    }

    nvs.offset = STORAGE_OFFSET;
    rc = flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &page_info);
    if (rc) {
        LOG_ERR("Cannot get flash page info: %d", rc);
        return rc;
    }
    nvs.sector_size  = page_info.size;
    nvs.sector_count = STORAGE_SIZE / page_info.size;

    rc = nvs_mount(&nvs);
    if (rc) {
        LOG_ERR("NVS mount failed: %d", rc);
        return rc;
    }

    /* Recover event count from a previous run (power-cycle safe) */
    rc = nvs_read(&nvs, NVS_ID_EVENT_COUNT, &stored_count, sizeof(stored_count));
    if (rc != sizeof(stored_count)) {
        stored_count = 0;
    }

    storage_ok = true;

    /* Rough capacity: NVS keeps one sector free; ~8 bytes ATE per entry */
    uint32_t usable = (nvs.sector_count - 1) * nvs.sector_size;
    uint32_t per_event = sizeof(struct stored_event) + 16;
    LOG_INF("[STORAGE] NVS mounted: %u KB partition, %u sectors of %u B",
            (unsigned)(STORAGE_SIZE / 1024),
            (unsigned)nvs.sector_count, (unsigned)nvs.sector_size);
    LOG_INF("[STORAGE] %u event(s) already stored, ~%u total capacity",
            stored_count, (unsigned)(usable / per_event));
    return 0;
}

static int store_event(int64_t session_id)
{
    if (!storage_ok) {
        LOG_ERR("[STORAGE] Not mounted, event lost.");
        return -ENODEV;
    }
    if (storage_full || stored_count >= MAX_STORED_EVENTS) {
        LOG_WRN("[STORAGE] Full (%u events). Event DISCARDED. "
                "Dump & erase to continue.", stored_count);
        storage_full = true;
        led_full_set(true);
        return -ENOSPC;
    }

    evt_buf.session_id = session_id;
    evt_buf.n_samples  = FULL_WINDOW_SAMPLES;
    evt_buf.reserved   = 0;

    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        struct sensor_record *r = &event_payload[i];
        evt_buf.s[i].x_raw = (int16_t)CLAMP(r->x * 16384.0, -32768.0, 32767.0);
        evt_buf.s[i].y_raw = (int16_t)CLAMP(r->y * 16384.0, -32768.0, 32767.0);
        evt_buf.s[i].z_raw = (int16_t)CLAMP(r->z * 16384.0, -32768.0, 32767.0);
        evt_buf.s[i].p_centi_hpa = (int32_t)(r->p_hpa * 100.0);
    }

    ssize_t rc = nvs_write(&nvs, NVS_ID_EVENT_BASE + stored_count,
                           &evt_buf, sizeof(evt_buf));
    if (rc < 0) {
        if (rc == -ENOSPC) {
            LOG_WRN("[STORAGE] Flash full. Event DISCARDED. "
                    "Dump & erase to continue.");
            storage_full = true;
            led_full_set(true);
        } else {
            LOG_ERR("[STORAGE] Write failed: %d", (int)rc);
        }
        return (int)rc;
    }

    stored_count++;
    nvs_write(&nvs, NVS_ID_EVENT_COUNT, &stored_count, sizeof(stored_count));

    LOG_INF("[STORAGE] Event stored (%u total, %d bytes).",
            stored_count, (int)rc);
    return 0;
}

/* Dump all stored events over the UART console in copy-paste-ready
 * CSV format. Each event is printed as:
 *
 *   ----- 32866 -----
 *   timestamp_ms,acc_x_g,acc_y_g,acc_z_g,pressure_delta_hpa
 *   -1500,0.02,-0.98,0.11,0.00
 *   ...
 *
 * Capture the terminal output (or use your terminal's log-to-file
 * feature), paste into a .csv, done. Dumping does NOT erase --
 * you can dump again if the capture failed. */
static void dump_all_events(void)
{
    if (!storage_ok) {
        LOG_ERR("[STORAGE] Not mounted, nothing to dump.");
        return;
    }

    led_dump_set(true);
    printk("\n===== FALL LOGGER DUMP BEGIN (%u events) =====\n", stored_count);

    for (uint16_t n = 0; n < stored_count; n++) {
        ssize_t rc = nvs_read(&nvs, NVS_ID_EVENT_BASE + n,
                              &evt_buf, sizeof(evt_buf));
        if (rc != sizeof(evt_buf)) {
            printk("# ERROR: read event %u failed (%d)\n", n, (int)rc);
            continue;
        }

        printk("----- %lld -----\n", evt_buf.session_id);
        printk("timestamp_ms,acc_x_g,acc_y_g,acc_z_g,pressure_hpa\n");

        for (int i = 0; i < evt_buf.n_samples; i++) {
            printk("%d,%.2f,%.2f,%.2f,%.2f\n",
                   (i - HALF_WINDOW_SAMPLES) * 20,
                   evt_buf.s[i].x_raw / 16384.0,
                   evt_buf.s[i].y_raw / 16384.0,
                   evt_buf.s[i].z_raw / 16384.0,
                   evt_buf.s[i].p_centi_hpa / 100.0);
            /* Small breather so we don't overrun the UART at 115200 */
            if ((i % 25) == 24) {
                k_sleep(K_MSEC(5));
            }
        }
        k_sleep(K_MSEC(20));
    }

    printk("===== FALL LOGGER DUMP END =====\n\n");
    led_dump_set(false);
    LOG_INF("[STORAGE] Dump complete. Hold button >= 2s to erase.");
}

static void erase_all_events(void)
{
    if (!storage_ok) {
        return;
    }

    LOG_WRN("[STORAGE] ERASING all stored events...");
    int rc = nvs_clear(&nvs);
    if (rc) {
        LOG_ERR("[STORAGE] nvs_clear failed: %d", rc);
        return;
    }
    /* nvs_clear unmounts the filesystem -- mount it again */
    rc = nvs_mount(&nvs);
    if (rc) {
        LOG_ERR("[STORAGE] re-mount after erase failed: %d", rc);
        storage_ok = false;
        return;
    }

    stored_count = 0;
    storage_full = false;
    led_full_set(false);

    /* Visual confirmation: blink red+blue 3x */
    for (int i = 0; i < 3; i++) {
        gpio_pin_set_dt(&led_red, 1);
        gpio_pin_set_dt(&led_blue, 1);
        k_sleep(K_MSEC(150));
        gpio_pin_set_dt(&led_red, 0);
        gpio_pin_set_dt(&led_blue, 0);
        k_sleep(K_MSEC(150));
    }
    LOG_INF("[STORAGE] Erased. Ready for new events.");
}

/* =====================================================================
 * MQTT PUBLISHING -- DISABLED (offline flash logging)
 * ===================================================================== */
#if 0
static int publish_training_chunk(int64_t session_id, int chunk_index, int start_idx, int sample_count)
{
    if (!network_active || !mqtt_is_connected) {
        LOG_WRN("Network offline, dropping chunk %d.", chunk_index);
        return -ENOTCONN;
    }

    char payload[4096];
    int offset = snprintf(payload, sizeof(payload),
                          "{\"session_id\":%lld,\"chunk\":%d,\"data\":[",
                          session_id, chunk_index);

    for (int i = 0; i < sample_count; i++) {
        const char *comma = (i == sample_count - 1) ? "" : ",";
        struct sensor_record *r = &event_payload[start_idx + i];

        char p_buf[16];
        if (r->press_valid) {
            snprintf(p_buf, sizeof(p_buf), "%.2f", r->p_hpa);
        } else {
            strcpy(p_buf, "null");
        }

        offset += snprintf(payload + offset, sizeof(payload) - offset,
                           "{\"t\":%lld,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f,\"dp\":%s}%s",
                           r->timestamp, r->x, r->y, r->z, p_buf, comma);

        if (offset >= sizeof(payload) - 5) {
            LOG_WRN("Payload buffer nearly full, truncating.");
            break;
        }
    }

    snprintf(payload + offset, sizeof(payload) - offset, "]}");

    struct mqtt_publish_param param = { 0 };
    static uint16_t next_msg_id = 1;
    param.message_id = next_msg_id++;
    if (next_msg_id == 0) next_msg_id = 1;
    param.message.payload.data = (uint8_t *)payload;
    param.message.payload.len = strlen(payload);
    param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    param.message.topic.topic.utf8 = (uint8_t *)MQTT_PUB_TOPIC;
    param.message.topic.topic.size = strlen(MQTT_PUB_TOPIC);

    LOG_INF("Publishing Event Chunk %d/3 (%d bytes)...", chunk_index, param.message.payload.len);
    return mqtt_helper_publish(&param);
}
#endif

/* =====================================================================
 * BUTTON: DUMP (short press) / ERASE (hold >= 2s)
 *
 * (The old graceful-LTE-shutdown behavior is preserved below in a
 *  #if 0 block for when the modem comes back.)
 * ===================================================================== */
static void handle_button_press(void)
{
    /* Decide short press vs. hold: sample the pin for up to 2s */
    int held_ms = 0;
    while (gpio_pin_get_dt(&user_button) > 0 && held_ms < 2000) {
        k_sleep(K_MSEC(50));
        held_ms += 50;
    }

    if (held_ms >= 2000) {
        erase_all_events();
        /* Wait for release so we don't re-trigger */
        while (gpio_pin_get_dt(&user_button) > 0) {
            k_sleep(K_MSEC(50));
        }
    } else {
        dump_all_events();
    }
}

#if 0
static void handle_button_press_lte(void)
{
    if (!network_active) {
        LOG_INF("Button pressed, but network is already offline. Ignoring.");
        return;
    }

    LOG_INF("Button pressed: shutting down network gracefully...");
    network_active = false;   /* stop any new publishes immediately */

    /* 1. Cleanly close the MQTT session (sends DISCONNECT packet) */
    if (mqtt_is_connected) {
        mqtt_helper_disconnect();
        /* Wait up to 5 s for the broker teardown to complete */
        for (int i = 0; i < 50 && mqtt_is_connected; i++) {
            k_sleep(K_MSEC(100));
        }
    }

    /* 2. Gracefully detach from the network and power the modem down */
    int err = lte_lc_power_off();
    if (err) {
        LOG_ERR("lte_lc_power_off failed: %d", err);
    } else {
        LOG_INF("LTE modem powered off cleanly.");
    }

    LOG_INF("Device is now OFFLINE. Events will still be captured but not uploaded.");
}
#endif

/* =====================================================================
 * LOW-G EVENT HANDLER
 * ===================================================================== */
static void handle_low_g_event(struct bmi2_dev *dev)
{
    uint16_t int_status = 0;
    float past_p[16], future_p[16];

    int8_t rslt = bmi2_get_int_status(&int_status, dev);
    if (rslt != BMI2_OK || !(int_status & BMI270_LEGACY_LOW_G_STATUS_MASK)) {
        return;
    }

    /* CRASH FIX: we are now busy for several seconds. Mask the IMU
     * interrupt at the GPIO level so continued motion cannot queue up
     * spurious events that re-enter this handler on a freshly-flushed
     * (nearly empty) FIFO. */
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_DISABLE);
    led_ready_set(false);   /* LED off = busy, not ready for a new measurement */

    memset(event_payload, 0, sizeof(event_payload));
    int64_t session_id = k_uptime_get();
    LOG_INF("=== LOW-G TRIGGERED (Session: %lld) ===", session_id);

    /* 1: Capture PAST */
    uint16_t past_acc_n   = read_bmi_half_window(event_payload, 0, "PAST");
    uint16_t past_press_n = read_pressure_fifo(past_p, 16);

    /* 2: Flush & Wait 1.65s so FIFO fully has 75 fresh samples */
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();

    k_sleep(K_MSEC(1650));

    /* 3: Capture FUTURE */
    uint16_t future_acc_n = read_bmi_half_window(event_payload, HALF_WINDOW_SAMPLES, "FUTURE");

    /* Diagnostics -- immediately after accel read */
    uint8_t aps_now = 0, pwr_now = 0, pwr_conf_now = 0, fifo_cfg_now = 0;
    bmi2_get_adv_power_save(&aps_now, dev);
    bmi2_get_regs(0x7D, &pwr_now, 1, dev);
    bmi2_get_regs(0x7C, &pwr_conf_now, 1, dev);
    i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, &fifo_cfg_now);
    LOG_INF("Post-FUTURE: aps=%u pwr=0x%02x conf=0x%02x fifo1=0x%02x (hdr=%d acc=%d)",
            aps_now, pwr_now, pwr_conf_now, fifo_cfg_now,
            !!(fifo_cfg_now & (1<<4)), !!(fifo_cfg_now & (1<<6)));

    /* Force-refresh accel state before next event */
    bmi2_set_adv_power_save(BMI2_DISABLE, dev);

    /* Read pressure -- ONCE only */
    uint16_t future_press_n = read_pressure_fifo(future_p, 16);

    LOG_INF("Captured: past_acc=%d/75 future_acc=%d/75 past_p=%d future_p=%d",
            past_acc_n, future_acc_n, past_press_n, future_press_n);

    if ((past_acc_n + future_acc_n) == FULL_WINDOW_SAMPLES) {
        align_and_pad_pressure(past_p, past_press_n, future_p, future_press_n);

        /* OFFLINE MODE: store to internal flash instead of MQTT */
        LOG_INF("Storing Event Data to internal flash...");
        store_event(session_id);

#if 0   /* --- Original MQTT upload path, disabled --- */
        LOG_INF("Publishing Event Data to Server...");
        publish_training_chunk(session_id, 1, 0, 50);
        k_sleep(K_MSEC(500));   /* let modem recover, let caps recharge */
        publish_training_chunk(session_id, 2, 50, 50);
        k_sleep(K_MSEC(500));
        publish_training_chunk(session_id, 3, 100, 50);
        LOG_INF("Upload Complete.");
#endif
    } else {
        LOG_WRN("FIFO read short: past=%d future=%d, discarding event.",
                past_acc_n, future_acc_n);
    }

    /* (Removed the 2s modem tail-current wait -- no modem running.) */

    /* Clean state before rearming */
    i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
    flush_pressure_fifo();

    LOG_INF("Refilling FIFOs (1.6s)...");
    k_sleep(K_MSEC(1600));

    /* CRASH FIX v2: rearm without the latched-INT deadlock.
     *
     * The BMI270 INT pin is LATCHED (stays high until int_status is
     * read) but our GPIO trigger is EDGE-to-active. If low-g re-fires
     * in the window between clearing the status and enabling the edge
     * (e.g. you're picking the device up at that instant), the pin is
     * already high when the edge arms -> no rising edge ever comes ->
     * permanent deadlock with the green LED on.
     *
     * Fix: (1) clear the latch until the pin is verified low,
     *      (2) arm the edge,
     *      (3) if the pin snuck high again, kick the semaphore
     *          manually so the handler runs instead of deadlocking.
     */
    for (int i = 0; i < 5; i++) {
        bmi2_get_int_status(&int_status, dev);
        if (gpio_pin_get_dt(&bmi_int) == 0) {
            break;
        }
        k_sleep(K_MSEC(10));
    }
    k_sem_reset(&bmi_irq_sem);
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_EDGE_TO_ACTIVE);
    if (gpio_pin_get_dt(&bmi_int) > 0) {
        LOG_WRN("INT already latched at rearm -- forcing handler run.");
        k_sem_give(&bmi_irq_sem);
    }

    led_ready_set(true);   /* LED on = ready for a new measurement */
    LOG_INF(">>> Ready for next event. <<<");
}

/* =====================================================================
 * MAIN
 * ===================================================================== */
int main(void)
{
    int8_t rslt;
    struct bmi2_dev dev = { 0 };

    LOG_INF("==========================================================");
    LOG_INF(" Fall Logger: OFFLINE Flash Logging Mode (no LTE/MQTT) ");
    LOG_INF("==========================================================");
    LOG_INF(" Button: short press = dump JSON over UART");
    LOG_INF("         hold >= 2s  = erase stored events");

    if (!i2c_is_ready_dt(&bmi_i2c) || !i2c_is_ready_dt(&icp_i2c)) {
        LOG_ERR("I2C bus not ready"); return 0;
    }

    /* --- LED & BUTTON INIT --- */
    if (gpio_is_ready_dt(&led_red) && gpio_is_ready_dt(&led_green) && gpio_is_ready_dt(&led_blue)) {
        gpio_pin_configure_dt(&led_red,   GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&led_blue,  GPIO_OUTPUT_INACTIVE);
    } else {
        LOG_ERR("LED GPIOs not ready");
    }

    if (gpio_is_ready_dt(&user_button)) {
        gpio_pin_configure_dt(&user_button, GPIO_INPUT);
        gpio_pin_interrupt_configure_dt(&user_button, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&button_cb, button_isr_handler, BIT(user_button.pin));
        gpio_add_callback(user_button.port, &button_cb);
    } else {
        LOG_ERR("Button GPIO not ready");
    }

    LOG_INF("Waiting 5s for power rails to stabilize...");
    k_sleep(K_MSEC(5000));

    /* --- FLASH STORAGE INIT (replaces modem init) --- */
    if (init_storage() != 0) {
        LOG_ERR("Storage init failed -- events will NOT be saved!");
        /* keep running so sensors can still be tested via logs */
    }

/* =====================================================================
 * MODEM + MQTT INIT -- DISABLED (offline flash logging).
 * Re-enable this block, the modem Kconfigs in prj.conf, and the
 * publish path in handle_low_g_event() when the battery is attached.
 * ===================================================================== */
#if 0
    char resp[128] = {0};

    LOG_INF("Initializing modem library...");
    if (nrf_modem_lib_init() != 0) {
        return -1;
    }

    int err;

    /* --- Modem is in CFUN=0 here. Do ALL config now. --- */

    err = nrf_modem_at_printf("AT+CGDCONT=0,\"IP\",\"iBASIS.iot\"");
    if (err) {
        LOG_ERR("CGDCONT set failed: %d", err);
    }
    memset(resp, 0, sizeof(resp));
    nrf_modem_at_cmd(resp, sizeof(resp), "AT+CGDCONT?");
    LOG_INF("APN readback: %s", resp);

    /* NOTE: the XEMPR "+10 dBm TX cap" was removed -- %XEMPR only
     * supports 0.5/1 dB reduction, so it never did anything. The
     * real fix is power delivery (battery + VBUS limit). */

    /* Band lock: band 20 (800 MHz). Comment out if your operator does
     * not serve LTE-M on band 20 at your location. */
    err = nrf_modem_at_printf("AT%%XBANDLOCK=1,\"10000000000000000000\"");
    if (err) {
        LOG_ERR("XBANDLOCK failed: %d", err);
    }

    /* Early warning of supply collapse at 3.0 V. */
    err = nrf_modem_at_printf("AT%%XPOFWARN=1,30");
    if (err) {
        LOG_WRN("XPOFWARN failed: %d", err);
    }

    /* Relaxed rescan: retry network search every 3600 s. */
    err = nrf_modem_at_printf("AT%%PERIODICSEARCHCONF=0,0,0,0,\"1,3600\"");
    if (err) {
        LOG_WRN("PERIODICSEARCHCONF failed: %d", err);
    }

    memset(resp, 0, sizeof(resp));
    nrf_modem_at_cmd(resp, sizeof(resp), "AT%%XSYSTEMMODE?");
    LOG_INF("System mode: %s", resp);

    /* Go online exactly once. */
    LOG_INF("Connecting to LTE network...");
    err = lte_lc_connect_async(lte_handler);
    if (err) {
        LOG_ERR("lte_lc_connect_async failed: %d", err);
        return -1;
    }
    k_sem_take(&lte_connected, K_FOREVER);

    struct mqtt_helper_cfg config = {
        .cb = {
            .on_connack = on_mqtt_connack,
            .on_disconnect = on_mqtt_disconnect,
        },
    };
    mqtt_helper_init(&config);

    struct mqtt_helper_conn_params conn_params = {
        .hostname.ptr = MQTT_BROKER_HOSTNAME,
        .hostname.size = strlen(MQTT_BROKER_HOSTNAME),
        .device_id.ptr = CLIENT_ID,
        .device_id.size = strlen(CLIENT_ID),
    };

    LOG_INF("Connecting to MQTT Broker...");
    mqtt_helper_connect(&conn_params);
    k_sem_take(&mqtt_connected_sem, K_FOREVER);
#endif  /* End of disabled modem/MQTT init */

    /* --- ICP-20100 INIT --- */
    LOG_INF("[INIT] Configuring ICP-20100 (25Hz Continuous)...");
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_MODE_SELECT, 0x08);

    /* WARM-UP FIX: the first ~14 conversions after starting a
     * measurement mode are invalid (~700 hPa settling transient).
     * The old code flushed the FIFO IMMEDIATELY after mode select --
     * before any conversion existed -- so the junk was produced
     * afterwards and sat in the 16-deep FIFO (which stops accepting
     * data when full) until the FIRST event read it out as its "past"
     * pressure window. Wait for the warm-up samples to actually be
     * produced (1.2 s at 25 Hz = ~30 conversions), THEN flush them. */
    LOG_INF("[INIT] ICP-20100 warm-up (1.2s)...");
    k_sleep(K_MSEC(1200));
    flush_pressure_fifo();

    /* --- BMI270 INIT --- */
    dev.intf = BMI2_I2C_INTF;
    dev.read = bmi2_i2c_read;
    dev.write = bmi2_i2c_write;
    dev.delay_us = bmi2_delay_us;
    dev.intf_ptr = NULL;
    dev.read_write_len = 32;

    rslt = bmi270_legacy_init(&dev);
    if (rslt != BMI2_OK) return 0;

    bmi2_set_adv_power_save(BMI2_DISABLE, &dev);
    k_msleep(5);

    struct bmi2_int_pin_config int_pin_cfg = { 0 };
    int_pin_cfg.pin_type = BMI2_INT1;
    int_pin_cfg.int_latch = BMI2_INT_LATCH;
    int_pin_cfg.pin_cfg[0].lvl = BMI2_INT_ACTIVE_HIGH;
    int_pin_cfg.pin_cfg[0].od  = BMI2_INT_PUSH_PULL;
    int_pin_cfg.pin_cfg[0].output_en = BMI2_INT_OUTPUT_ENABLE;
    int_pin_cfg.pin_cfg[0].input_en  = BMI2_INT_INPUT_DISABLE;
    bmi2_set_int_pin_config(&int_pin_cfg, &dev);

    gpio_pin_configure_dt(&bmi_int, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&bmi_int, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&bmi_int_cb, bmi_isr_handler, BIT(bmi_int.pin));
    gpio_add_callback(bmi_int.port, &bmi_int_cb);

    uint8_t sens_list[2] = { BMI2_ACCEL, BMI2_LOW_G };
    bmi270_legacy_sensor_enable(sens_list, 2, &dev);

    struct bmi2_sens_config accel_cfg = { .type = BMI2_ACCEL };
    bmi270_legacy_get_sensor_config(&accel_cfg, 1, &dev);
    accel_cfg.cfg.acc.odr         = BMI2_ACC_ODR_50HZ;
    accel_cfg.cfg.acc.range       = BMI2_ACC_RANGE_2G;
    accel_cfg.cfg.acc.bwp         = BMI2_ACC_NORMAL_AVG4;
    accel_cfg.cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;
    bmi270_legacy_set_sensor_config(&accel_cfg, 1, &dev);

    struct bmi2_sens_config low_g_cfg = { .type = BMI2_LOW_G };
    bmi270_legacy_get_sensor_config(&low_g_cfg, 1, &dev);
    low_g_cfg.cfg.low_g.threshold  = 0x0300;
    low_g_cfg.cfg.low_g.hysteresis = 0x0100;
    low_g_cfg.cfg.low_g.duration   = 0x0002;
    bmi270_legacy_set_sensor_config(&low_g_cfg, 1, &dev);

    struct bmi2_sens_int_config low_g_int = {
        .type = BMI2_LOW_G, .hw_int_pin = BMI2_INT1
    };
    bmi270_legacy_map_feat_int(&low_g_int, 1, &dev);

    LOG_INF("[INIT] Configuring BMI FIFO...");
    configure_bmi_fifo();

    /* Let FIFOs fill before accepting interrupts */
    k_sleep(K_MSEC(1600));

    /* Same latched-INT protection as the rearm path: clear any low-g
     * latched during the fill, verify the pin is low, and if it is
     * still high after arming, run the handler instead of deadlocking. */
    {
        uint16_t boot_int_status = 0;

        for (int i = 0; i < 5; i++) {
            bmi2_get_int_status(&boot_int_status, &dev);
            if (gpio_pin_get_dt(&bmi_int) == 0) {
                break;
            }
            k_sleep(K_MSEC(10));
        }
        k_sem_reset(&bmi_irq_sem);
        if (gpio_pin_get_dt(&bmi_int) > 0) {
            LOG_WRN("INT latched at boot arm -- forcing handler run.");
            k_sem_give(&bmi_irq_sem);
        }
    }

    led_ready_set(true);   /* Green LED on = ready for first measurement */
    LOG_INF(">>> READY. Offline logging active. Waiting for drops. <<<");

    /* Poll on BOTH the IMU interrupt semaphore and the button semaphore
     * so the button works even while we sit idle waiting for events. */
    struct k_poll_event events[2] = {
        K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &bmi_irq_sem, 0),
        K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                        K_POLL_MODE_NOTIFY_ONLY,
                                        &button_sem, 0),
    };

    while (1) {
        int rc = k_poll(events, 2, K_MSEC(600));

        /* FIFO FRESHNESS FIX: the ICP-20100 FIFO is only 16 samples
         * deep and stops accepting data once full. Without this, it
         * fills within ~640 ms of the last flush and then holds those
         * same STALE samples forever -- so the "past" pressure window
         * of an event could be minutes old. Flushing on every idle
         * timeout guarantees the FIFO only ever contains the most
         * recent <= 600 ms of data when a drop hits. */
        if (rc == -EAGAIN) {
            flush_pressure_fifo();
            continue;
        }

        if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
            k_sem_take(&button_sem, K_NO_WAIT);
            handle_button_press();
        }

        if (events[0].state == K_POLL_STATE_SEM_AVAILABLE) {
            k_sem_take(&bmi_irq_sem, K_NO_WAIT);
            handle_low_g_event(&dev);
        }

        events[0].state = K_POLL_STATE_NOT_READY;
        events[1].state = K_POLL_STATE_NOT_READY;
    }

    return 0;
}
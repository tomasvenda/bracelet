#include <nrf_modem_at.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

/* --- Modem & MQTT Headers --- */
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <net/mqtt_helper.h>

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

static uint8_t bmi_fifo_buffer[HALF_WINDOW_BYTES];
static uint8_t icp_fifo_buffer[16 * 6];

/* =====================================================================
 * MQTT CONFIGURATION
 * ===================================================================== */
#define MQTT_PUB_TOPIC "bracelet/prototype_pcb/data/training"
#define CLIENT_ID "prototype_pcb"
#define MQTT_BROKER_HOSTNAME "20.251.201.46"

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(mqtt_connected_sem, 0, 1);
static bool mqtt_is_connected = false;

struct sensor_record {
    int64_t timestamp; /* relative to impact (t=0) */
    double x, y, z;
    double p_hpa;
    bool press_valid;
};

static struct sensor_record event_payload[FULL_WINDOW_SAMPLES];

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

BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr);
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr);
void bmi2_delay_us(uint32_t period, void *intf_ptr);

/* =====================================================================
 * NETWORK & MQTT CALLBACKS
 * ===================================================================== */
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

    if (strcmp(label, "FUTURE") == 0) {
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
 * DATA PROCESSING & MQTT PUBLISHING
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
        event_payload[idx].p_hpa = past_p[i] * 10.0f;      /* line 267 */
        event_payload[idx].p_hpa = future_p[i] * 10.0f;    /* line 277 */
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

    double baseline_p = event_payload[0].p_hpa;
    for (int i = 0; i < FULL_WINDOW_SAMPLES; i++) {
        event_payload[i].p_hpa = event_payload[i].p_hpa - baseline_p;
    }
}

static int publish_training_chunk(int64_t session_id, int chunk_index, int start_idx, int sample_count)
{
    if (!mqtt_is_connected) {
        LOG_WRN("MQTT not connected, dropping chunk %d.", chunk_index);
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

/* =====================================================================
 * MAIN
 * ===================================================================== */
int main(void)
{
    int8_t rslt;
    struct bmi2_dev dev = { 0 };
    uint16_t int_status = 0;

    LOG_INF("==========================================================");
    LOG_INF(" Fall Logger: Event-Driven MQTT Training Pipeline ");
    LOG_INF("==========================================================");

    if (!i2c_is_ready_dt(&bmi_i2c) || !i2c_is_ready_dt(&icp_i2c)) {
        LOG_ERR("I2C bus not ready"); return 0;
    }

    LOG_INF("Waiting 5s for power rails to stabilize...");
    k_sleep(K_MSEC(5000));
    char resp[128] = {0};

    LOG_INF("Initializing modem library...");
    if (nrf_modem_lib_init() != 0) return -1;
    nrf_modem_at_printf("AT+CGDCONT=0,\"IP\",\"iBASIS.iot\"");
    nrf_modem_at_cmd(resp, sizeof(resp), "AT+CGDCONT?");
    LOG_INF("APN readback: %s", resp);
    /* Cap modem TX power BEFORE connecting to reduce current spikes on weak signal.
    * +10 dBm = ~10x less peak current than +23 dBm.
    * Requires modem in OFFLINE mode to change.
    */
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_OFFLINE);

    /* AT%XEMPR: cap max TX power to +10 dBm across all bands */
    nrf_modem_at_printf("AT%%XEMPR=1,1,4,10");

    /* AT%XBANDLOCK: lock to band 20 (800 MHz — best indoor coverage in EU).
    * NOTE: only enable this if your operator supports band 20 at your location.
    * If unsure, comment this line out for now.
    */
    nrf_modem_at_printf("AT%%XBANDLOCK=1,\"10000000000000000000\"");

    

    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);

    /* Reduce retry aggressiveness — helps prevent current bursts after registration failures */
    nrf_modem_at_printf("AT%%XPOFWARN=1,30");   /* Enable early warning of power failures */
    nrf_modem_at_printf("AT+CFUN=4");           /* Offline first */
    k_sleep(K_MSEC(1000));

    /* Set search timing profile: less aggressive rescan */
    nrf_modem_at_printf("AT%%PERIODICSEARCHCONF=0,0,0,\"2,3600\"");

    nrf_modem_at_printf("AT+CFUN=1");           /* Back online */
    
    k_sleep(K_MSEC(500));

    /* Query which system information the cell is broadcasting */
    memset(resp, 0, sizeof(resp));
    nrf_modem_at_cmd(resp, sizeof(resp), "AT%%XSYSTEMMODE?");
    LOG_INF("System mode: %s", resp);

    /* Scan for available cells and networks */
    memset(resp, 0, sizeof(resp));
    nrf_modem_at_cmd(resp, sizeof(resp), "AT%%NCELLMEAS");
    LOG_INF("Cell measurement started: %s", resp);

    LOG_INF("Connecting to LTE network...");
    lte_lc_connect_async(lte_handler);
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

    /* --- ICP-20100 INIT --- */
    LOG_INF("[INIT] Configuring ICP-20100 (25Hz Continuous)...");
    i2c_reg_write_byte_dt(&icp_i2c, ICP20100_REG_MODE_SELECT, 0x08);
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
    k_sem_reset(&bmi_irq_sem);

    LOG_INF(">>> READY. System Online. Waiting for drops. <<<");

    float past_p[16], future_p[16];

    while (1) {
        k_sem_take(&bmi_irq_sem, K_FOREVER);

        rslt = bmi2_get_int_status(&int_status, &dev);
        if (rslt != BMI2_OK || !(int_status & BMI270_LEGACY_LOW_G_STATUS_MASK)) {
            k_sem_reset(&bmi_irq_sem);
            continue;
        }

        memset(event_payload, 0, sizeof(event_payload));
        int64_t session_id = k_uptime_get();
        LOG_INF("=== LOW-G TRIGGERED (Session: %lld) ===", session_id);

        /* 1: Capture PAST */
        uint16_t past_acc_n   = read_bmi_half_window(event_payload, 0, "PAST");
        uint16_t past_press_n = read_pressure_fifo(past_p, 16);

        /* 2: Flush & Wait 1.65s so FIFO fully has 75 fresh samples */
        i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
        flush_pressure_fifo();
        k_sem_reset(&bmi_irq_sem);

        k_sleep(K_MSEC(1650));   /* extended from 1550 to guarantee full FUTURE window */

        /* 3: Capture FUTURE */
        uint16_t future_acc_n = read_bmi_half_window(event_payload, HALF_WINDOW_SAMPLES, "FUTURE");

        /* Diagnostics — immediately after accel read */
        uint8_t aps_now = 0, pwr_now = 0, pwr_conf_now = 0, fifo_cfg_now = 0;
        bmi2_get_adv_power_save(&aps_now, &dev);
        bmi2_get_regs(0x7D, &pwr_now, 1, &dev);
        bmi2_get_regs(0x7C, &pwr_conf_now, 1, &dev);
        i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_FIFO_CONFIG_1, &fifo_cfg_now);
        LOG_INF("Post-FUTURE: aps=%u pwr=0x%02x conf=0x%02x fifo1=0x%02x (hdr=%d acc=%d)",
                aps_now, pwr_now, pwr_conf_now, fifo_cfg_now,
                !!(fifo_cfg_now & (1<<4)), !!(fifo_cfg_now & (1<<6)));

        /* Force-refresh accel state before next event */
        bmi2_set_adv_power_save(BMI2_DISABLE, &dev);

        /* Read pressure — ONCE only */
        uint16_t future_press_n = read_pressure_fifo(future_p, 16);

        LOG_INF("Captured: past_acc=%d/75 future_acc=%d/75 past_p=%d future_p=%d",
                past_acc_n, future_acc_n, past_press_n, future_press_n);

        if ((past_acc_n + future_acc_n) == FULL_WINDOW_SAMPLES) {
            align_and_pad_pressure(past_p, past_press_n, future_p, future_press_n);

            LOG_INF("Publishing Event Data to Server...");
            publish_training_chunk(session_id, 1, 0, 50);
            k_sleep(K_MSEC(500));   /* let modem recover, let caps recharge */
            publish_training_chunk(session_id, 2, 50, 50);
            k_sleep(K_MSEC(500));
            publish_training_chunk(session_id, 3, 100, 50);

            LOG_INF("Upload Complete.");
        } else {
            LOG_WRN("FIFO read short: past=%d future=%d, discarding event.",
                    past_acc_n, future_acc_n);
        }

        /* Clean state before rearming */
        bmi2_get_int_status(&int_status, &dev);
        i2c_reg_write_byte_dt(&bmi_i2c, BMI270_REG_CMD, BMI270_CMD_FIFO_FLUSH);
        flush_pressure_fifo();
        k_sem_reset(&bmi_irq_sem);

        LOG_INF("Upload Complete.");
        k_sleep(K_MSEC(2000));    /* let modem finish tail current before FIFO refill */
        LOG_INF("Refilling FIFOs (1.6s)...");
        k_sleep(K_MSEC(1600));
        LOG_INF(">>> Ready for next event. <<<");
    }

    return 0;
}
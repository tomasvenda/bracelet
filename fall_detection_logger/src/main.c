#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <net/mqtt_helper.h>


LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

/* --- Hardware Definitions --- */
#define BMI270_NODE DT_NODELABEL(bmi270)
#define ICP20100_NODE DT_NODELABEL(icp20100)
#define LDO2_NODE DT_NODELABEL(npm1300_ldsw2)

/* --- Dedicated Training Topic --- */
#define MQTT_PUB_TOPIC "bracelet/prototype_pcb/data/training"
#define CLIENT_ID "prototype_pcb"
#define MQTT_BROKER_HOSTNAME "20.251.201.46"

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(mqtt_connected_sem, 0, 1);
static bool mqtt_is_connected = false;

/* --- Buffer Configuration --- */
#define MAX_SAMPLES 50 /* 1 second of data at 50Hz */

/* Bootstrap security code — set this in prj.conf as CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE */
#ifndef CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE
#define CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE "987654"
#endif

struct sensor_record {
    int64_t timestamp;
    double x, y, z;
    double p_hpa;
    bool acc_valid;
    bool press_valid;
};

static struct sensor_record sample_buffer[MAX_SAMPLES];

/* =====================================================================
 * PRE-MAIN BOOT SEQUENCE (Power up sensors)
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

/* =====================================================================
 * CALLBACKS
 * ===================================================================== */
static void lte_handler(const struct lte_lc_evt *const evt)
{
    if (evt->type == LTE_LC_EVT_NW_REG_STATUS) {
        if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
            (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            LOG_INF("Connected to LTE network!");
            k_sem_give(&lte_connected);
        } else if (evt->nw_reg_status == LTE_LC_NW_REG_REGISTRATION_DENIED) {
            LOG_ERR("Network rejected us!");
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
 * HELPER: PUBLISH BUFFERED JSON
 * ===================================================================== */
static int publish_training_chunk(int64_t session_id, int chunk_index, int sample_count)
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
        struct sensor_record *r = &sample_buffer[i];

        char x_buf[16], y_buf[16], z_buf[16], p_buf[16];

        if (r->acc_valid) {
            snprintf(x_buf, sizeof(x_buf), "%.2f", r->x);
            snprintf(y_buf, sizeof(y_buf), "%.2f", r->y);
            snprintf(z_buf, sizeof(z_buf), "%.2f", r->z);
        } else {
            strcpy(x_buf, "null");
            strcpy(y_buf, "null");
            strcpy(z_buf, "null");
        }

        if (r->press_valid) {
            snprintf(p_buf, sizeof(p_buf), "%.2f", r->p_hpa);
        } else {
            strcpy(p_buf, "null");
        }

        offset += snprintf(payload + offset, sizeof(payload) - offset,
                           "{\"t\":%lld,\"x\":%s,\"y\":%s,\"z\":%s,\"p\":%s}%s",
                           r->timestamp, x_buf, y_buf, z_buf, p_buf, comma);

        if (offset >= sizeof(payload) - 5) {
            LOG_WRN("Payload buffer nearly full, truncating chunk %d at sample %d.",
                    chunk_index, i);
            break;
        }
    }

    snprintf(payload + offset, sizeof(payload) - offset, "]}");

    struct mqtt_publish_param param = { 0 };
    static uint16_t next_msg_id = 1;

    param.message_id = next_msg_id++;
    if (next_msg_id == 0) {
        next_msg_id = 1;
    }
    param.message.payload.data = (uint8_t *)payload;
    param.message.payload.len = strlen(payload);
    param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    param.message.topic.topic.utf8 = (uint8_t *)MQTT_PUB_TOPIC;
    param.message.topic.topic.size = strlen(MQTT_PUB_TOPIC);

    LOG_INF("Publishing Chunk %d/10 (%d bytes)...", chunk_index, param.message.payload.len);
    return mqtt_helper_publish(&param);
}

/* =====================================================================
 * MAIN APPLICATION
 * ===================================================================== */
int main(void)
{
    int err;
    LOG_INF("=== ML Data Collection Pipeline Booting ===");

    /* --- 1. Init Sensors --- */
    const struct device *const bmi_dev = DEVICE_DT_GET(BMI270_NODE);
    const struct device *const icp_dev = DEVICE_DT_GET(ICP20100_NODE);

    if (!device_is_ready(bmi_dev) || !device_is_ready(icp_dev)) {
        LOG_ERR("Sensors failed to initialize. Check hardware.");
        return -1;
    }

    struct sensor_value full_scale = { .val1 = 2, .val2 = 0 };
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    LOG_INF("Sensors ready.");

    /* --- 2. Init Modem & Connect LTE --- */
    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) return -1;


    LOG_INF("Connecting to LTE network...");
    lte_lc_connect_async(lte_handler);
    k_sem_take(&lte_connected, K_FOREVER);

    /* --- 3. Init & Connect MQTT --- */
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
    
    LOG_INF("Connecting to MQTT Broker at %s...", MQTT_BROKER_HOSTNAME);
    mqtt_helper_connect(&conn_params);
    k_sem_take(&mqtt_connected_sem, K_FOREVER);

    /* --- 4. The 10-Second Wait --- */
    LOG_INF("==================================================");
    LOG_INF("DATA COLLECTION STARTING IN 10 SECONDS...");
    LOG_INF("Get into position!");
    LOG_INF("==================================================");
    k_sleep(K_SECONDS(10));

    /* --- 5. The 10-Second Data Collection Sequence --- */
    struct sensor_value acc[3];
    struct sensor_value pressure;
    struct sensor_value sampling_freq = { .val1 = 100, .val2 = 0 };
    struct sensor_value oversampling = { .val1 = 1, .val2 = 0 };
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
    
    /* Create a unique session ID based on system uptime */
    int64_t session_id = k_uptime_get(); 
    
    LOG_INF(">>> STARTING 10-SECOND RECORDING (Session: %lld) <<<", session_id);

    for (int chunk = 1; chunk <= 10; chunk++) {

        for (int sample = 0; sample < MAX_SAMPLES; sample++) {

            int64_t now = k_uptime_get() - session_id;
            struct sensor_record *r = &sample_buffer[sample];
            r->timestamp = now;

            int acc_err = sensor_sample_fetch(bmi_dev);
            if (acc_err == 0) {
                sensor_channel_get(bmi_dev, SENSOR_CHAN_ACCEL_XYZ, acc);
                r->x = sensor_value_to_double(&acc[0]);
                r->y = sensor_value_to_double(&acc[1]);
                r->z = sensor_value_to_double(&acc[2]);
                r->acc_valid = true;
            } else {
                LOG_WRN("BMI270 fetch failed at t=%lld, err=%d", now, acc_err);
                r->acc_valid = false;
            }

            k_usleep(500);

            static double last_valid_pressure = 0.0;
            static bool have_valid_pressure = false;

            if (sample % 2 == 0) {
                int press_err = sensor_sample_fetch(icp_dev);
                if (press_err == 0) {
                    sensor_channel_get(icp_dev, SENSOR_CHAN_PRESS, &pressure);
                    r->p_hpa = sensor_value_to_double(&pressure) * 10.0;
                    r->press_valid = true;
                    last_valid_pressure = r->p_hpa;
                    have_valid_pressure = true;
                } else {
                    LOG_WRN("ICP20100 fetch failed at t=%lld, err=%d", now, press_err);
                    r->press_valid = false;
                }
            } else {
                /* Carry forward the last known pressure instead of leaving it null */
                r->p_hpa = last_valid_pressure;
                r->press_valid = have_valid_pressure;
            }
            k_sleep(K_MSEC(20));
        }

        /* 1 second has passed, publish the chunk */
        publish_training_chunk(session_id, chunk, MAX_SAMPLES);
    }

    /* --- 6. Shutdown --- */
    LOG_INF("==================================================");
    LOG_INF("DATA COLLECTION COMPLETE.");
    LOG_INF("==================================================");
    
    LOG_INF("Disconnecting safely to preserve network status...");
    mqtt_helper_disconnect();
    k_sleep(K_MSEC(200)); /* let the socket close gracefully */
    lte_lc_power_off();
    
    while (1) {
        k_sleep(K_SECONDS(10));
    }

    return 0;
}
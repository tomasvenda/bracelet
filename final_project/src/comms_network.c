#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/zbus/zbus.h>

#include <modem/modem_info.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <net/mqtt_helper.h>
#include <nrf_modem_at.h>
#include <modem/modem_info.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "comms.h"
#include "events.h"

LOG_MODULE_REGISTER(comms_network, LOG_LEVEL_INF);

/* ------------------------------------------------------------------
 * CONSTANTS & CONFIG
 * ------------------------------------------------------------------ */
#define MQTT_TOPIC "bracelet/prototype_pcb/data"
#define CLIENT_ID "prototype_pcb"
#define MQTT_SUB_TOPIC "bracelet/prototype_pcb/response"
#define MQTT_AGNSS_TOPIC "bracelet/prototype_pcb/agnss"

#ifndef CONFIG_MQTT_BROKER_HOSTNAME
#define CONFIG_MQTT_BROKER_HOSTNAME "20.251.201.46"
#endif

#ifndef CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE
#define CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE "987654"
#endif

/* ------------------------------------------------------------------
 * STATE & STRUCTS
 * ------------------------------------------------------------------ */
// static bool bootstrap_sent = false;
static bool mqtt_is_connected = false;
static bool in_backoff;
static void lte_backoff_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(lte_backoff_work, lte_backoff_fn);
static volatile comms_raw_cb_t raw_cb;  


static struct mqtt_topic sub_topics[] = {
    { .topic = { .utf8 = (uint8_t *)MQTT_SUB_TOPIC,
                 .size = sizeof(MQTT_SUB_TOPIC) - 1 },
      .qos = MQTT_QOS_1_AT_LEAST_ONCE },
    { .topic = { .utf8 = (uint8_t *)MQTT_AGNSS_TOPIC,
                 .size = sizeof(MQTT_AGNSS_TOPIC) - 1 },
      .qos = MQTT_QOS_1_AT_LEAST_ONCE },
};

static struct mqtt_subscription_list sub_list = {
    .list = sub_topics,
    .list_count = ARRAY_SIZE(sub_topics),
    .message_id = 1234,
};

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(mqtt_connected_sem, 0, 1);

/* ------------------------------------------------------------------
 * LTE LOGIC
 * ------------------------------------------------------------------ */
static void lte_backoff_fn(struct k_work *work)
{
    if (!in_backoff) {
        LOG_WRN("Registration denied: modem offline, 2 min backoff.");
        lte_lc_offline();
        in_backoff = true;
        k_work_schedule(&lte_backoff_work, K_MINUTES(2));
    } else {
        LOG_INF("Backoff over: modem back to normal mode.");
        lte_lc_normal();
        in_backoff = false;
    }
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
    if (evt->type == LTE_LC_EVT_NW_REG_STATUS) {
        if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
            (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            in_backoff = false;
            k_work_cancel_delayable(&lte_backoff_work);
            LOG_INF("LTE Network registered");
            k_sem_give(&lte_connected);
        } else if (evt->nw_reg_status == LTE_LC_NW_REG_REGISTRATION_DENIED) {
            LOG_ERR("Cell tower rejected connection! Scheduling backoff.");
            k_work_schedule(&lte_backoff_work, K_NO_WAIT);
        }
    }
}

/* ------------------------------------------------------------------
 * MQTT CALLBACKS
 * ------------------------------------------------------------------ */
void comms_set_raw_response_cb(comms_raw_cb_t cb) { raw_cb = cb; }

static void on_mqtt_connack(enum mqtt_conn_return_code return_code, bool session_present)
{
    if (return_code == MQTT_CONNECTION_ACCEPTED) {
        LOG_INF("Connected to MQTT broker");
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

static void on_mqtt_publish(struct mqtt_helper_buf topic, struct mqtt_helper_buf payload)
{
    
    if (topic.size == sizeof(MQTT_AGNSS_TOPIC) - 1 &&
        memcmp(topic.ptr, MQTT_AGNSS_TOPIC, topic.size) == 0) {
        if (raw_cb) {
            raw_cb(payload.ptr, payload.size);
        }
        return;
    }
    
    char buf[64] = {0};
    size_t len = MIN((size_t)payload.size, sizeof(buf) - 1);

    memcpy(buf, payload.ptr, len);

    if (strcmp(buf, "wifi_successful") == 0) {
        k_event_post(&app_events, WIFI_EVT_SUCCESS);
        return;
    }
    else if (strcmp(buf, "wifi_failed") == 0) {
        k_event_post(&app_events, WIFI_EVT_FAIL);
        return;
    }

    struct bracelet_event event;
    if (strstr(buf, "located_home")) {
        event.type = EVENT_SERVER_REPLY_HOME;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
        k_event_post(&app_events, LOC_EVT_HOME);
    } 
    else if (strstr(buf, "located_away")) {
        event.type = EVENT_SERVER_REPLY_AWAY;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
        k_event_post(&app_events, LOC_EVT_AWAY);
    }
    
    else if (strstr(buf, "\"ack\": true")) {
        event.type = EVENT_SERVER_ACK_ALERT;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
    }
}

static int mqtt_connect_and_subscribe(k_timeout_t connack_timeout)
{
    struct mqtt_helper_conn_params conn_params = {
        .hostname.ptr = CONFIG_MQTT_BROKER_HOSTNAME,
        .hostname.size = strlen(CONFIG_MQTT_BROKER_HOSTNAME),
        .device_id.ptr = CLIENT_ID,
        .device_id.size = strlen(CLIENT_ID),
    };

    int err = mqtt_helper_connect(&conn_params);
    if (err) {
        LOG_ERR("MQTT connect failed: %d", err);
        return -EIO;
    }

    k_sem_reset(&mqtt_connected_sem);   /* drop any stale token */
    if (k_sem_take(&mqtt_connected_sem, connack_timeout) != 0) {
        LOG_ERR("MQTT CONNACK timeout");
        return -ETIMEDOUT;
    }

    err = mqtt_helper_subscribe(&sub_list);
    if (err) {
        LOG_ERR("MQTT subscribe failed: %d", err);
        return err;
    }
    return 0;
}

/* ------------------------------------------------------------------
 * PUBLIC NETWORK APIS
 * ------------------------------------------------------------------ */
int comms_mqtt_ensure_connected(void)
{
    if (mqtt_is_connected) return 0;

    LOG_INF("MQTT is disconnected. Reconnecting...");

    lte_lc_normal();

    enum lte_lc_nw_reg_status reg_status;
    lte_lc_nw_reg_status_get(&reg_status);

    if (reg_status != LTE_LC_NW_REG_REGISTERED_HOME &&
        reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING) {
        LOG_INF("Waiting for LTE...");
        k_sem_reset(&lte_connected);
        if (k_sem_take(&lte_connected, K_SECONDS(120)) != 0) {
            LOG_ERR("LTE Reconnect failed!");
            return -ENETUNREACH;
        }
    }

    return mqtt_connect_and_subscribe(K_SECONDS(15));
}

int comms_mqtt_publish(const char *payload)
{
    if (comms_mqtt_ensure_connected() != 0) return -ENOTCONN;

    struct mqtt_publish_param param = { 0 };
    param.message.payload.data = (uint8_t *)payload;
    param.message.payload.len = strlen(payload);
    param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    param.message_id = mqtt_helper_msg_id_get();
    param.message.topic.topic.utf8 = (uint8_t *)MQTT_TOPIC;
    param.message.topic.topic.size = strlen(MQTT_TOPIC);

    return mqtt_helper_publish(&param);
}

void comms_mqtt_disconnect(void)
{
    if (mqtt_is_connected) {
        mqtt_helper_disconnect();
        k_msleep(300);
        mqtt_is_connected = false;
    }
}


/* =========================================
 * LTE Wrappers for comms_core.c
 * ========================================= */
void comms_lte_wake(void) { lte_lc_normal(); }
void comms_lte_sleep(void) { lte_lc_offline(); }

void comms_lte_gnss_mode(void)
{
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
}

void comms_lte_normal_mode(void)
{
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_GNSS);
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_LTE);
}


void comms_safe_disconnect(void)
{    
    if (mqtt_is_connected) {
        LOG_INF("Disconnecting MQTT...");
        mqtt_helper_disconnect();
        k_msleep(1000); 
        mqtt_is_connected = false;
    }

    LOG_INF("Detaching from LTE network...");
    lte_lc_power_off(); 
    k_sem_reset(&lte_connected); 
}

/* ------------------------------------------------------------------
 * INITIALIZATION
 * ------------------------------------------------------------------ */
int comms_network_init(void)
{
    int err;

    err = modem_info_init();
    if (err) {
        LOG_ERR("Modem info init failed: %d", err);
        return err;
    }

    LOG_INF("Connecting to LTE network");
    err = lte_lc_connect_async(lte_handler);
    if (err) {
        LOG_ERR("LTE connect async failed, err %d", err);
        return -1;
    }

    if (k_sem_take(&lte_connected, K_MINUTES(5)) != 0) {
        LOG_ERR("LTE connection timed out. Is the SIM card valid?");
        return -ETIMEDOUT;
    } else {
        LOG_INF("LTE Connected successfully.");
    } 

    struct mqtt_helper_cfg config = {
        .cb = {
            .on_connack = on_mqtt_connack,
            .on_disconnect = on_mqtt_disconnect,
            .on_publish = on_mqtt_publish,
        },
    };
    
    err = mqtt_helper_init(&config);
    if (err) return err;

    LOG_INF("Connecting to MQTT Broker...");
    return mqtt_connect_and_subscribe(K_SECONDS(20));
}
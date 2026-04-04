#include <zephyr/kernel.h>              
#include <zephyr/logging/log.h>         
#include <string.h>                     

#include <modem/nrf_modem_lib.h> 
#include <modem/lte_lc.h>        
#include <net/mqtt_helper.h> 

#include "lte_service.h"

LOG_MODULE_REGISTER(lte_service, LOG_LEVEL_INF);

static K_SEM_DEFINE(lte_connected, 0, 1);               
static K_SEM_DEFINE(mqtt_connected_sem, 0, 1);
static bool mqtt_is_connected = false;

#define CLIENT_ID_LEN sizeof("prototype_1")
static uint8_t client_id[CLIENT_ID_LEN] = "prototype_1";

static void lte_handler(const struct lte_lc_evt *const evt)
{
     switch (evt->type) {
     case LTE_LC_EVT_NW_REG_STATUS:
        if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
            (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            break;
        }
        LOG_INF("Network registration status: %s",
                evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
                "Connected - home network" : "Connected - roaming");
        k_sem_give(&lte_connected);
        break;
     default:
             break;
     }
}

static void on_mqtt_connack(enum mqtt_conn_return_code return_code, bool session_present)
{
    if (return_code == MQTT_CONNECTION_ACCEPTED) {
        LOG_INF("Connected to MQTT broker");
        LOG_INF("Client ID: %s", client_id);
        mqtt_is_connected = true;
        k_sem_give(&mqtt_connected_sem);
    } else {
        LOG_WRN("Connection to broker not established, return_code: %d", return_code);
    }
}

static void on_mqtt_disconnect(int result)
{
    LOG_INF("MQTT client disconnected: %d", result);
    mqtt_is_connected = false;
}

static void on_mqtt_publish(struct mqtt_helper_buf topic, struct mqtt_helper_buf payload)
{
    LOG_INF("Received unexpected payload: %.*s on topic: %.*s", 
        payload.size, payload.ptr, topic.size, topic.ptr);
}

static int mqtt_ensure_connected(void)
{
    if (mqtt_is_connected) {
        return 0; // Already connected
    }

    // This is expected to happen after every gnss fix because gnss turns off the LTE antenna
    LOG_INF("MQTT is not connected. Attempting reconnection...");
    
    // This first part checks if lte was down and waits until lte is reconnected
    enum lte_lc_nw_reg_status reg_status;
    int err = lte_lc_nw_reg_status_get(&reg_status);
    
    if (err == 0 && (reg_status != LTE_LC_NW_REG_REGISTERED_HOME && 
                     reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
        LOG_INF("LTE network not ready yet (status %d). Waiting for LTE connection...", reg_status);
        
        // Clear any old semaphore states
        k_sem_reset(&lte_connected);
        
        // Wait for the lte_handler to signal a successful registration
        if (k_sem_take(&lte_connected, K_SECONDS(120)) != 0) {
            LOG_ERR("LTE connection timed out. Cannot connect MQTT.");
            return -ENETUNREACH;
        }
        LOG_INF("LTE re-attached successfully. Proceeding with MQTT connection.");
    }

    // Second part just reconnects to MQTT
    struct mqtt_helper_conn_params conn_params = {
        .hostname.ptr = CONFIG_MQTT_BROKER_HOSTNAME,
        .hostname.size = strlen(CONFIG_MQTT_BROKER_HOSTNAME),
        .device_id.ptr = (char *)client_id,
        .device_id.size = strlen(client_id),
    };
    
    err = mqtt_helper_connect(&conn_params);
    if (err) {
        LOG_ERR("Failed to trigger reconnect, err %d", err);
        return err;
    }

    LOG_INF("Waiting for reconnect confirmation...");
    if (k_sem_take(&mqtt_connected_sem, K_SECONDS(10)) != 0) {
        LOG_ERR("MQTT Reconnection timed out");
        return -ETIMEDOUT;
    }

    return 0;
}

int lte_mqtt_publish_str(const char *payload)
{
    if (mqtt_ensure_connected() != 0) {
        return -ENOTCONN;
    }

    struct mqtt_publish_param mqtt_param = { 0 };

    mqtt_param.message.payload.data = (uint8_t *)payload;
    mqtt_param.message.payload.len = strlen(payload);
    mqtt_param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    mqtt_param.message_id = mqtt_helper_msg_id_get();
    
    mqtt_param.message.topic.topic.utf8 = (uint8_t *)CONFIG_MQTT_PUB_TOPIC;
    mqtt_param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC);

    int err = mqtt_helper_publish(&mqtt_param);
    if (err) {
        LOG_WRN("Failed to send payload, err: %d", err);
        return err;
    }

    LOG_INF("Successfully published: %s", payload);
    return 0;
}

int lte_mqtt_init(void)
{
    int err;

    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) return err;
    
    LOG_INF("Connecting to LTE network... (This may take a few seconds)");
    err = lte_lc_connect_async(lte_handler);                        
    if (err) return err;

    /* Wait for LTE to connect */
    k_sem_take(&lte_connected, K_FOREVER);                          
    LOG_INF("Connected to LTE network!");

    // K_SEM_DEFINE(mqtt_connected_sem, 0, 1);

    struct mqtt_helper_cfg config = {
        .cb = {
            .on_connack = on_mqtt_connack,
            .on_disconnect = on_mqtt_disconnect,
            .on_publish = on_mqtt_publish,
        },
    };
    err = mqtt_helper_init(&config);
    if (err) return err;

    struct mqtt_helper_conn_params conn_params = {
        .hostname.ptr = CONFIG_MQTT_BROKER_HOSTNAME,
        .hostname.size = strlen(CONFIG_MQTT_BROKER_HOSTNAME),
        .device_id.ptr = (char *)client_id,
        .device_id.size = strlen(client_id),
    };
    
    LOG_INF("Connecting to Azure MQTT Broker...");
    err = mqtt_helper_connect(&conn_params);
    if (err) return err;

    /* Wait for MQTT connection */
    LOG_INF("Waiting for initial MQTT connection...");
    
    // Give MQTT time to establish the handshake before we start bombing it with data
    if (k_sem_take(&mqtt_connected_sem, K_SECONDS(20)) != 0) {
        LOG_ERR("Initial MQTT connection failed!");
        return -ETIMEDOUT;
    }
    
    return 0;
}
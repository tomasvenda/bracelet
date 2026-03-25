#include <zephyr/kernel.h>              
#include <zephyr/logging/log.h>         
#include <string.h>                     

#include <modem/nrf_modem_lib.h> 
#include <modem/lte_lc.h>        
#include <net/mqtt_helper.h> 

#include "lte_service.h"

LOG_MODULE_REGISTER(lte_service, LOG_LEVEL_INF);

static K_SEM_DEFINE(lte_connected, 0, 1);               

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
    } else {
        LOG_WRN("Connection to broker not established, return_code: %d", return_code);
    }
}

static void on_mqtt_disconnect(int result)
{
    LOG_INF("MQTT client disconnected: %d", result);
}

static void on_mqtt_publish(struct mqtt_helper_buf topic, struct mqtt_helper_buf payload)
{
    LOG_INF("Received unexpected payload: %.*s on topic: %.*s", 
        payload.size, payload.ptr, topic.size, topic.ptr);
}

int lte_mqtt_publish_str(const char *payload)
{
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

    /* Give MQTT a second to establish the handshake before we start bombing it with data */
    k_sleep(K_SECONDS(2)); 
    return 0;
}
/*
	This file implements the communication between Thingy and out Azure Cloud Server via MQTT
*/

#include <zephyr/kernel.h>				// Core Zephyr OS functions
#include <zephyr/logging/log.h> 		// Zephyr logging library
#include <string.h>						// C library for string operations

/* Networking and Modem includes */
#include <modem/nrf_modem_lib.h> 
#include <modem/lte_lc.h>        
#include <net/mqtt_helper.h> 

#include <nrf_modem_at.h> 				// Not used currently, will be used in final implementation to use AT+CGSN to ask the modem for its IMEI number

LOG_MODULE_REGISTER(Bracelet_Tracker, LOG_LEVEL_INF);

static K_SEM_DEFINE(lte_connected, 0, 1);				// Semaphore that will block the main thread until LTE is successfully connected

/* Hardcoded the Client ID size and value for this prototype, maybe for final version will be based on the IMEI */
#define CLIENT_ID_LEN sizeof("prototype_1")
static uint8_t client_id[CLIENT_ID_LEN] = "prototype_1";

/* 
 	Background callback function triggered automatically by the modem library.
 	It is called asynchronously whenever the LTE connection status changes 
 	examples: the connection drops or the modem successfully registeres
 */
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
	case LTE_LC_EVT_RRC_UPDATE:
		LOG_INF("RRC mode: %s", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ?
				"Connected" : "Idle");
		break;
     default:
             break;
     }
}

/* 
 	Main setup function to initialize and power on the cellular modem.
 	It is called exactly once by the main() function at the start of the program,
 	and it blocks execution until a successful cellular connection is established.
 */
static int modem_configure(void)
{
	int err;

	LOG_INF("Initializing modem library");
	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("Failed to initialize the modem library, error: %d", err);
		return err;
	}
	
	LOG_INF("Connecting to LTE network");
	err = lte_lc_connect_async(lte_handler);						// This asks the modem to look for networks async
	if (err) {
		LOG_ERR("Error in lte_lc_connect_async, error: %d", err);
		return err;
	}

	k_sem_take(&lte_connected, K_FOREVER);							// This blocks the execution until the semaphore is released from lte_handler function 				
	LOG_INF("Connected to LTE network");

	return 0;
}


// This function gets the ID using the IMEI number and asking the modem using AT commands, may be useful for final version

// static int client_id_get(char * buffer, size_t buffer_len)
// {
// 	/* STEP 9.1 Define the function to generate the client id */
// 		int len;
// 	int err;
// 	char imei_buf[CGSN_RESPONSE_LENGTH];

// 	if (!buffer || buffer_len == 0) {
// 		LOG_ERR("Invalid buffer parameters");
// 		return -EINVAL;
// 	}

// 	if (strlen(CONFIG_MQTT_SAMPLE_CLIENT_ID) > 0) {
// 		len = snprintk(buffer, buffer_len, "%s",
// 			 CONFIG_MQTT_SAMPLE_CLIENT_ID);
// 	if ((len < 0) || (len >= buffer_len)) {
// 		LOG_ERR("Failed to format client ID from config, error: %d", len);
// 		return -EMSGSIZE;
// 	}
// 	LOG_DBG("client_id = %s", buffer);
// 	return 0;
// 	}

// 	err = nrf_modem_at_cmd(imei_buf, sizeof(imei_buf), "AT+CGSN");
// 	if (err) {
// 		LOG_ERR("Failed to obtain IMEI, error: %d", err);
// 		return err;
// 	}

// 	/* Validate IMEI length before null termination */
// 	if (IMEI_LEN >= sizeof(imei_buf)) {
// 		LOG_ERR("IMEI_LEN exceeds buffer size");
// 		return -EINVAL;
// 	}

// 	imei_buf[IMEI_LEN] = '\0';

// 	len = snprintk(buffer, buffer_len, "nrf-%.*s", IMEI_LEN, imei_buf);
// 	if ((len < 0) || (len >= buffer_len)) {
// 		LOG_ERR("Failed to format client ID from IMEI, error: %d", len);
// 		return -EMSGSIZE;
// 	}

// 	LOG_DBG("client_id = %s", buffer);

// 	return 0;
// }


/* 
	Function to publish data to the Azure MQTT broker.
 */
static int publish(uint8_t *data, size_t len)
{
	int err;
	struct mqtt_publish_param mqtt_param;

	mqtt_param.message.payload.data = data;
	mqtt_param.message.payload.len = len;
	mqtt_param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
	mqtt_param.message_id = mqtt_helper_msg_id_get();
	
	/* Use the topic we defined in prj.conf */
	mqtt_param.message.topic.topic.utf8 = CONFIG_MQTT_PUB_TOPIC;
	mqtt_param.message.topic.topic.size = strlen(CONFIG_MQTT_PUB_TOPIC);
	
	mqtt_param.dup_flag = 0;
	mqtt_param.retain_flag = 0;

	err = mqtt_helper_publish(&mqtt_param);
	if (err) {
		LOG_WRN("Failed to send payload, err: %d", err);
		return err;
	}

	LOG_INF("Published message: \"%.*s\" on topic: \"%.*s\"", 
		mqtt_param.message.payload.len,
		mqtt_param.message.payload.data,
		mqtt_param.message.topic.topic.size,
		mqtt_param.message.topic.topic.utf8);
	
	return 0;
}

/* 
 * Callback triggered when the MQTT broker accepts or rejects our connection request.
 */
static void on_mqtt_connack(enum mqtt_conn_return_code return_code, bool session_present)
{
	if (return_code == MQTT_CONNECTION_ACCEPTED) {
		LOG_INF("Connected to MQTT broker");
		LOG_INF("Client ID: %s", client_id);
	} else {
		LOG_WRN("Connection to broker not established, return_code: %d", return_code);
	}
}

/* 
 * Callback triggered when the MQTT client disconnects from the broker.
 */
static void on_mqtt_disconnect(int result)
{
	LOG_INF("MQTT client disconnected: %d", result);
}

/* 
 * Callback triggered if the device somehow accidentally receives an MQTT message.
 * Since this version only publishes, we just log it and ignore it.
 */
static void on_mqtt_publish(struct mqtt_helper_buf topic, struct mqtt_helper_buf payload)
{
	LOG_INF("Received unexpected payload: %.*s on topic: %.*s", 
		payload.size, payload.ptr, 
		topic.size, topic.ptr);
}

int main(void)
{
	int err;

	LOG_INF("Starting the server communication main.c");

	/* Initialize the cellular modem and wait for LTE connection */
	err = modem_configure();
	if (err) {
		LOG_ERR("Failed to configure the modem, error: %d", err);
		return 0;
	}

	/* Initialize the MQTT helper library with the custom callbacks */
	struct mqtt_helper_cfg config = {
		.cb = {
			.on_connack = on_mqtt_connack,
			.on_disconnect = on_mqtt_disconnect,
			.on_publish = on_mqtt_publish,
		},
	};
	err = mqtt_helper_init(&config);
	if (err) {
		LOG_ERR("Failed to initialize MQTT helper, error: %d", err);
		return 0;
	}

	// /* STEP 9.3 - Generate the client ID */
	// err = client_id_get(client_id, sizeof(client_id));
    // if (err) {
    //     LOG_ERR("Failed to get client ID, error: %d", err);
    //     return 0;
    // }

	/* Establish a connection to the MQTT broker hosted on our beautiful Azure Server */
	struct mqtt_helper_conn_params conn_params = {
		.hostname.ptr = CONFIG_MQTT_BROKER_HOSTNAME,
		.hostname.size = strlen(CONFIG_MQTT_BROKER_HOSTNAME),
		.device_id.ptr = (char *)client_id,
		.device_id.size = strlen(client_id),
	};
	
	err = mqtt_helper_connect(&conn_params);
	if (err) {
		LOG_ERR("Failed to connect to MQTT, error code: %d", err);
		return 0;
	}

	/* Main Loop: Publish a test message every 10 seconds */
	int i = 1;
	char test_msg[128];

	while (1) {
		k_sleep(K_SECONDS(10));

		snprintf(test_msg, sizeof(test_msg), "Hello from Prototype_1, message %d", i);

		err = publish((uint8_t *)test_msg, strlen(test_msg));		// Message must always be a pointer because MQTT sends raw bytes
		if (err) {
			LOG_WRN("Failed to publish message %d", i);
		}
	}

	return 0;
}

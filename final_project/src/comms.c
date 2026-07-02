#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/zbus/zbus.h>
#include <nrf_modem_gnss.h>
#include <modem/modem_info.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <net/mqtt_helper.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "events.h"
#include "comms.h"

LOG_MODULE_REGISTER(comms_system, LOG_LEVEL_INF);

/* ------------------------------------------------------------------
 * HARDWARE BINDINGS & STATE
 * ------------------------------------------------------------------ */
/* Define the PMIC GPIO enable pin from the overlay alias */
#define PMIC_WIFI_EN_NODE DT_ALIAS(pmic_gpio1)
static const struct gpio_dt_spec pmic_wifi_en = GPIO_DT_SPEC_GET(PMIC_WIFI_EN_NODE, gpios);

static K_SEM_DEFINE(wifi_scan_sem, 0, 1);
static K_SEM_DEFINE(gnss_fix_sem, 0, 1);
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(mqtt_connected_sem, 0, 1);

K_EVENT_DEFINE(app_events); // event flag used purely for the Wi-Fi localisation handshake

#define MQTT_TOPIC "bracelet/prototype_pcb/data"
#define CLIENT_ID "prototype_pcb"

#define MQTT_SUB_TOPIC "bracelet/prototype_pcb/response"
/* List of topics the MQTT helper will subscribe to */
static struct mqtt_topic sub_topic = {
    .topic = {
        .utf8 = (uint8_t *)MQTT_SUB_TOPIC,
        .size = sizeof(MQTT_SUB_TOPIC) - 1,
    },
    .qos = MQTT_QOS_1_AT_LEAST_ONCE,
};

static struct mqtt_subscription_list sub_list = {
    .list = &sub_topic,
    .list_count = 1,
    .message_id = 1234
};

/* Fallback broker definition if not defined in Kconfig */
#ifndef CONFIG_MQTT_BROKER_HOSTNAME
#define CONFIG_MQTT_BROKER_HOSTNAME "20.251.201.46"
#endif

/* Bootstrap security code — set this in prj.conf as CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE */
#ifndef CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE
#define CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE "987654"
#endif

static bool bootstrap_sent = false;

static bool mqtt_is_connected = false;
static const char *current_status = "ok"; 

#define MAX_APS_PER_SCAN 10
static struct ap_data_t { char mac[18]; int8_t rssi; } best_aps[MAX_APS_PER_SCAN];
static uint8_t current_ap_count = 0;
static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static bool gnss_has_fix = false;

/* Dummy battery function - Replace with actual ADC logic later */
static int get_battery_level(void) { return 85; }

/* ------------------------------------------------------------------
 * EVENT HANDLERS (WIFI & GNSS)
 * ------------------------------------------------------------------ */
static void handle_wifi_scan_result(struct net_mgmt_event_callback *cb)
{
    const struct wifi_scan_result *entry = (const struct wifi_scan_result *)cb->info;
    char mac_string_buf[18];

    snprintf(mac_string_buf, sizeof(mac_string_buf), "%02X:%02X:%02X:%02X:%02X:%02X",
             entry->mac[0], entry->mac[1], entry->mac[2], 
             entry->mac[3], entry->mac[4], entry->mac[5]);

    for (int i = 0; i < current_ap_count; i++) {
        if (strcmp(best_aps[i].mac, mac_string_buf) == 0) return; /* Ignore duplicates */
    }

    if (current_ap_count < MAX_APS_PER_SCAN) {
        strcpy(best_aps[current_ap_count].mac, mac_string_buf);
        best_aps[current_ap_count].rssi = entry->rssi;
        current_ap_count++;
    } else {
        int weakest_idx = 0;
        for (int i = 1; i < MAX_APS_PER_SCAN; i++) {
            if (best_aps[i].rssi < best_aps[weakest_idx].rssi) weakest_idx = i;
        }
        if (entry->rssi > best_aps[weakest_idx].rssi) {
            strcpy(best_aps[weakest_idx].mac, mac_string_buf);
            best_aps[weakest_idx].rssi = entry->rssi;
        }
    }
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event, struct net_if *iface)
{
    if (mgmt_event == NET_EVENT_WIFI_SCAN_RESULT) {
        handle_wifi_scan_result(cb);
    } else if (mgmt_event == NET_EVENT_WIFI_SCAN_DONE) {
        k_sem_give(&wifi_scan_sem);
    }
}

static void gnss_event_handler(int event)
{
    if (event == NRF_MODEM_GNSS_EVT_PVT) {
        if (nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT) == 0) {
            if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
                LOG_INF("GNSS FIX ACQUIRED! Lat: %.6f, Lon: %.6f", last_pvt.latitude, last_pvt.longitude);
                gnss_has_fix = true;
                k_sem_give(&gnss_fix_sem);
            }
        }
    }
}

/* ------------------------------------------------------------------
 * EVENT HANDLERS (LTE & MQTT)
 * ------------------------------------------------------------------ */
static void lte_handler(const struct lte_lc_evt *const evt)
{
    if (evt->type == LTE_LC_EVT_NW_REG_STATUS) {
        if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
            (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            LOG_INF("LTE Network registered");
            k_sem_give(&lte_connected);
        } else if (evt->nw_reg_status == LTE_LC_NW_REG_REGISTRATION_DENIED) {
            LOG_ERR("Cell twers rejected connection! Turning off modem to prevent FPLMN lock.");
            lte_lc_offline();
            k_sleep(K_MINUTES(2));
            lte_lc_normal();
        }
    }
}

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
    /* Convert payload to a null-terminated string for easy comparison */
    char buf[64] = {0};
    size_t len = MIN((size_t)payload.size, sizeof(buf) - 1);
    memcpy(buf, payload.ptr, len);

    LOG_INF("MQTT RX: %s  |  topic: %.*s", buf, topic.size, topic.ptr);
    
    /* 1. Localization Responses (via k_event) */
    if (strcmp(buf, "wifi_successful") == 0) {
        k_event_post(&app_events, EVENT_SERVER_WIFI_SUCCESS);
        return;
    } 
    else if (strcmp(buf, "wifi_failed") == 0) {
        k_event_post(&app_events, EVENT_SERVER_WIFI_FAIL);
        return;
    }

    /* 2. ZBUS FSM Events (if the payload is JSON) */
    struct bracelet_event event;
    if (strstr(buf, "located_home")) {
        event.type = EVENT_SERVER_REPLY_HOME;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
    } 
    else if (strstr(buf, "located_away")) {
        event.type = EVENT_SERVER_REPLY_AWAY;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
    }
    else if (strstr(buf, "\"ack\": true")) {
        event.type = EVENT_SERVER_ACK_ALERT;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
    }
}

static int mqtt_ensure_connected(void)
{
    if (mqtt_is_connected) return 0;

    LOG_INF("MQTT is disconnected. Reconnecting...");
    
    /* Ensure LTE is attached first */
    enum lte_lc_nw_reg_status reg_status;
    lte_lc_nw_reg_status_get(&reg_status);
    
    if (reg_status != LTE_LC_NW_REG_REGISTERED_HOME && reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING) {
        LOG_INF("Waiting for LTE...");
        k_sem_reset(&lte_connected);
        if (k_sem_take(&lte_connected, K_SECONDS(120)) != 0) return -ENETUNREACH;
    }

    struct mqtt_helper_conn_params conn_params = {
        .hostname.ptr = CONFIG_MQTT_BROKER_HOSTNAME,
        .hostname.size = strlen(CONFIG_MQTT_BROKER_HOSTNAME),
        .device_id.ptr = CLIENT_ID,
        .device_id.size = strlen(CLIENT_ID),
    };
    
    if (mqtt_helper_connect(&conn_params) != 0) return -EIO;

    if (k_sem_take(&mqtt_connected_sem, K_SECONDS(15)) != 0) return -ETIMEDOUT;
    return 0;
}

static int lte_mqtt_publish_str(const char *payload)
{
    if (mqtt_ensure_connected() != 0) return -ENOTCONN;

    struct mqtt_publish_param param = { 0 };
    param.message.payload.data = (uint8_t *)payload;
    param.message.payload.len = strlen(payload);
    param.message.topic.qos = MQTT_QOS_1_AT_LEAST_ONCE;
    param.message_id = mqtt_helper_msg_id_get();
    param.message.topic.topic.utf8 = (uint8_t *)MQTT_TOPIC;
    param.message.topic.topic.size = strlen(MQTT_TOPIC);

    return mqtt_helper_publish(&param);
}

static int comms_send_bootstrap(void)
{
    char payload[128];
    int written = snprintf(payload, sizeof(payload),
        "{\"security_code\":\"%s\",\"battery\":%d,\"status\":\"ok\"}",
        CONFIG_MQTT_BOOTSTRAP_SECURITY_CODE,
        get_battery_level());

    if (written < 0 || written >= (int)sizeof(payload)) {
        LOG_ERR("Bootstrap payload format failed");
        return -ENOMEM;
    }

    LOG_INF("Publishing bootstrap message: %s", payload);
    return lte_mqtt_publish_str(payload);
}


/* ------------------------------------------------------------------
 * LOCALIZATION TRIGGERS
 * ------------------------------------------------------------------ */
static int do_wifi_scan(void)
{   
    /*
    struct net_if *iface = net_if_get_default();
    if (!iface) return -ENOENT;

    current_ap_count = 0;
    struct wifi_scan_params params = { .scan_type = WIFI_SCAN_TYPE_ACTIVE, .dwell_time_active = 50 };

    if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, &params, sizeof(params))) return -EIO;
    k_sem_take(&wifi_scan_sem, K_SECONDS(10));
    */

    // Mocking the top 10 strongest APs from the provided network list
    current_ap_count = 10;
    
    // 84% Signal -> approx -58 dBm
    strcpy(best_aps[0].mac, "00:2a:10:20:c5:91");
    best_aps[0].rssi = -58; 
    strcpy(best_aps[1].mac, "00:2a:10:20:c5:92");
    best_aps[1].rssi = -58; 
    strcpy(best_aps[2].mac, "00:2a:10:20:c5:93");
    best_aps[2].rssi = -58; 

    // 83% Signal -> approx -59 dBm
    strcpy(best_aps[3].mac, "00:2a:10:13:6c:b2");
    best_aps[3].rssi = -59; 
    strcpy(best_aps[4].mac, "00:2a:10:13:6c:b3");
    best_aps[4].rssi = -59; 
    strcpy(best_aps[5].mac, "00:2a:10:13:6c:b1");
    best_aps[5].rssi = -59; 

    // 82% Signal -> approx -59 dBm
    strcpy(best_aps[6].mac, "00:2a:10:20:c5:9f");
    best_aps[6].rssi = -59; 

    // 81% Signal -> approx -60 dBm
    strcpy(best_aps[7].mac, "00:2a:10:20:c5:9d");
    best_aps[7].rssi = -60; 
    strcpy(best_aps[8].mac, "00:2a:10:20:c5:9c");
    best_aps[8].rssi = -60; 
    strcpy(best_aps[9].mac, "00:2a:10:20:c5:9e");
    best_aps[9].rssi = -60; 

    // Log for debugging
    LOG_INF("Wi-Fi scan bypassed. Loaded %d mock access points.", current_ap_count);

    return current_ap_count;
}

static int do_gnss_fix(void)
{
    gnss_has_fix = false;
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
    
    nrf_modem_gnss_fix_interval_set(0);    /* Single fix mode */
    nrf_modem_gnss_fix_retry_set(180);     /* 180s hardware timeout */
    nrf_modem_gnss_start();

    int res = k_sem_take(&gnss_fix_sem, K_SECONDS(180));
    
    nrf_modem_gnss_stop();
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_GNSS);
    return (res == 0) ? 0 : -ETIMEDOUT;
}

/* ------------------------------------------------------------------
 * BACKGROUND WORKER: LOCALIZATION WATERFALL
 * ------------------------------------------------------------------ */
static void perform_localization_work(struct k_work *work)
{
    static char payload[1024]; 
    int offset = 0;
    int batt = get_battery_level();
    int aps = 0;

    //LOG_INF("Starting localization. Suspending LTE to free RF front-end...");
    //lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);

    /* STEP 1: WI-FI */
    if (gpio_is_ready_dt(&pmic_wifi_en)) {
        gpio_pin_set_dt(&pmic_wifi_en, 1);
        k_msleep(100); /* Wait 100ms for PMIC voltage to stabilize */
    }
    
    struct net_if *iface = net_if_get_default();
    if (iface) {
        net_if_up(iface); 
        aps = do_wifi_scan();
        net_if_down(iface); 
    }
    
    if (gpio_is_ready_dt(&pmic_wifi_en)) {
        gpio_pin_set_dt(&pmic_wifi_en, 0); 
    }

        if (aps >= 3) {
        LOG_INF("Wi-Fi sufficient.");
        //LOG_INF("Wi-Fi sufficient. Reactivating LTE...");
        //lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
        
        /* Build the Wi-Fi Payload */
        offset = snprintf(payload, sizeof(payload), 
            "{\"status\":\"%s\", \"battery\":%d, \"wifi\":{\"accessPoints\":[", current_status, batt);
        for (int i = 0; i < aps; i++) {
            offset += snprintf(payload + offset, sizeof(payload) - offset, 
                "{\"macAddress\":\"%s\",\"signalStrength\":%d}%s", 
                best_aps[i].mac, best_aps[i].rssi, (i < aps - 1) ? "," : "");
        }
        snprintf(payload + offset, sizeof(payload) - offset, "]}}");
        
        /* 1. Publish immediately */
        LOG_INF("Publishing Wi-Fi Payload: %s", payload);
        lte_mqtt_publish_str(payload);

        /* 2. Set up event waiting logic */
        bool wifi_successful = false;
        
        /* Clear any stale flags from previous runs */
        k_event_clear(&app_events, EVENT_SERVER_WIFI_SUCCESS | EVENT_SERVER_WIFI_FAIL);
        
        /* Block the thread for up to 10 seconds */
        uint32_t events = k_event_wait(
            &app_events,
            EVENT_SERVER_WIFI_SUCCESS | EVENT_SERVER_WIFI_FAIL,
            false,
            K_SECONDS(10)
        );

        /* 3. Evaluate the result */
        if (events & EVENT_SERVER_WIFI_SUCCESS) {
            wifi_successful = true;
            LOG_INF("Server confirmed Wi-Fi localization -> Ending pipeline.");
        } else if (events & EVENT_SERVER_WIFI_FAIL) {
            LOG_INF("Server rejected Wi-Fi data.");
        } else {
            LOG_INF("Server response timeout (10s).");
        }

        /* 4. If successful, end the function completely */
        if (wifi_successful) {
            return; 
        }
    }

    else {
        /* If we reach here, Wi-Fi failed, timed out, or had < 3 APs */
        LOG_INF("Wi-Fi failed or timed out. Suspending LTE for GNSS (maybe not required)...");
        lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);

        /* STEP 2: GNSS */
        if (do_gnss_fix() == 0) {
            LOG_INF("GNSS Fix acquired. Reactivating LTE...");
            lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
            
            snprintf(payload, sizeof(payload), 
                "{\"status\":\"%s\", \"battery\":%d, \"gnss\":{\"lat\":%.6f, \"lon\":%.6f, \"accuracy\":%.1f}}", 
                current_status, batt, last_pvt.latitude, last_pvt.longitude, (double)last_pvt.accuracy);
        } 
        else {
            LOG_INF("GNSS Timeout. Reactivating LTE for Cell ID fallback...");
            lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
            
            /* STEP 3: LTE CELL ID */
            k_sem_reset(&lte_connected);
            if (k_sem_take(&lte_connected, K_SECONDS(60)) == 0) {
                struct modem_param_info modem_param = {0};
                modem_info_params_init(&modem_param);
                modem_info_params_get(&modem_param);
                
                char *cellid_str = modem_param.network.cellid_hex.value_string;
                uint32_t eci = (cellid_str && strlen(cellid_str) > 0) ? strtol(cellid_str, NULL, 16) : 0;
                uint16_t mcc = modem_param.network.mcc.value;
                uint16_t mnc = modem_param.network.mnc.value;
                uint32_t tac = modem_param.network.area_code.value;

                snprintf(payload, sizeof(payload), 
                    "{\"status\":\"%s\", \"battery\":%d, \"lte\":{\"mcc\":%d, \"mnc\":%d, \"tac\":%d, \"eci\":%u}}", 
                    current_status, batt, mcc, mnc, tac, eci);
            } else {
                LOG_ERR("LTE Reconnect failed. Fallback payload.");
                snprintf(payload, sizeof(payload), 
                    "{\"status\":\"%s\", \"battery\":%d}", current_status, batt);
            }
        }
    }

    /* Transmit via MQTT */
    LOG_INF("Publishing Payload: %s", payload);
    if (lte_mqtt_publish_str(payload) != 0) {
        LOG_ERR("Failed to publish! (TODO: Save to SPI Flash NVS here)");
    }
}
K_WORK_DEFINE(loc_work, perform_localization_work);

/* ------------------------------------------------------------------
 * PUBLIC FSM COMMANDS
 * ------------------------------------------------------------------ */
void comms_update_localization(void)
{
    //current_status = "ok";
    k_work_submit(&loc_work);
}

// Clears the current status from alert
void comms_clear_alert(void) {
    current_status = "ok";
}

void comms_send_alert(enum alert_reason reason)
{
    current_status = (reason == REASON_FALL_DETECTED) ? "fall" : "panic";
    k_work_submit(&loc_work);
}

/* ------------------------------------------------------------------
 * INITIALIZATION
 * ------------------------------------------------------------------ */
int comms_init(void)
{
    int err;

    /* Initialize the PMIC GPIO pin for Wi-Fi power to OFF */
    if (gpio_is_ready_dt(&pmic_wifi_en)) {
        gpio_pin_configure_dt(&pmic_wifi_en, GPIO_OUTPUT_INACTIVE);
    }

    /* Register Wi-Fi callbacks */
    static struct net_mgmt_event_callback wifi_cb;
    net_mgmt_init_event_callback(&wifi_cb, wifi_mgmt_event_handler, 
                                 NET_EVENT_WIFI_SCAN_RESULT | NET_EVENT_WIFI_SCAN_DONE);
    net_mgmt_add_event_callback(&wifi_cb);

    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) return err;

    /* Initialize Modem Info exactly once at boot */
    modem_info_init();

    /* Register GNSS callback */
    nrf_modem_gnss_event_handler_set(gnss_event_handler);
    
    LOG_INF("Connecting to LTE network... (This may take a few seconds)");
    err = lte_lc_connect_async(lte_handler);                        
    if (err) return err;

    /* Wait for LTE to connect */
    k_sem_take(&lte_connected, K_FOREVER);                          

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
        .device_id.ptr = CLIENT_ID,
        .device_id.size = strlen(CLIENT_ID),
    };
    
    LOG_INF("Connecting to MQTT Broker...");
    err = mqtt_helper_connect(&conn_params);
    if (err) return err;

    if (k_sem_take(&mqtt_connected_sem, K_SECONDS(20)) != 0) {
        LOG_ERR("Initial MQTT connection failed!");
        return -ETIMEDOUT;
    }

    if (!bootstrap_sent) {
        err = comms_send_bootstrap();
        if (err) {
            LOG_ERR("Bootstrap publish failed: %d", err);
            return err;
        }
        bootstrap_sent = true;
    }

    // Subscribing to the responses topic
    LOG_INF("Subscribing to %s...", MQTT_SUB_TOPIC);
    err = mqtt_helper_subscribe(&sub_list);
    if (err) {
        LOG_ERR("Failed to subscribe to MQTT topic: %d", err);
        return err;
    }
    LOG_INF("Subscribe SUCCESSFUL");
    return 0;
}
/*
* PART OF MASTER'S THESIS: Design of an End-to-End IoT System for Monitoring Vulnerable Users 
 
 
 * comms_core.c -- Localization orchestration and alert/status reporting.
 * Runs the Wi-Fi -> GNSS -> LTE-cell fallback waterfall on its own thread,
 * builds and publishes MQTT JSON payloads, and reports LOC_SUCCESS/
 * LOC_FAILURE back to the FSM. Also owns comms_init() top-level bring-up. 
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_if.h>
#include <zephyr/zbus/zbus.h>
#include <modem/modem_info.h>
#include <nrf_modem_gnss.h>
#include <zephyr/drivers/regulator.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "events.h"
#include "comms.h"
#include "sensors.h"

LOG_MODULE_REGISTER(comms_core, LOG_LEVEL_INF);

K_EVENT_DEFINE(app_events);

K_SEM_DEFINE(loc_start_sem, 0, 1);
static atomic_t loc_busy = ATOMIC_INIT(0);

static const char *current_status = "ok";

static const struct device *ldo1_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldo1));

/* ------------------------------------------------------------------
 * JSON PAYLOAD HELPERS
 * ------------------------------------------------------------------ */
static void send_status_payload(int batt) 
{
    char payload[128];
    snprintf(payload, sizeof(payload), "{\"status\":\"%s\", \"battery\":%d}", current_status, batt);
    LOG_INF("Publishing Status: %s", payload);
    comms_mqtt_publish(payload);
}

static void send_wifi_payload(int batt, int aps, const struct ap_data_t *best_aps)
{
    char payload[2048]; 
    int offset = 0;
    int remaining = sizeof(payload);
    
    offset += snprintf(payload + offset, remaining,
        "{\"status\":\"%s\", \"battery\":%d, \"wifi\":{\"accessPoints\":[", current_status, batt);
    remaining -= offset;

    for (int i = 0; i < aps; i++) {
        if (remaining < 64) {
            LOG_WRN("Payload buffer full! Truncating Wi-Fi list.");
            break; 
        }
        int written = snprintf(payload + offset, remaining,
            "{\"macAddress\":\"%s\",\"signalStrength\":%d}%s",
            best_aps[i].mac, best_aps[i].rssi, (i < aps - 1) ? "," : "");
        offset += written;
        remaining -= written;
    }

    snprintf(payload + offset, remaining, "]}}");
    LOG_INF("Publishing Wi-Fi Payload (%d bytes)", offset);
    comms_mqtt_publish(payload);
}

static void send_gnss_payload(int batt, const struct nrf_modem_gnss_pvt_data_frame *pvt)
{
    char payload[256];
    snprintf(payload, sizeof(payload),
        "{\"status\":\"%s\", \"battery\":%d, \"gnss\":{\"lat\":%.6f, \"lon\":%.6f, \"accuracy\":%.1f}}",
        current_status, batt, pvt->latitude, pvt->longitude, (double)pvt->accuracy);
    LOG_INF("Publishing GNSS Payload: %s", payload);
    comms_mqtt_publish(payload);
}

static void send_lte_payload(int batt)
{
    if (comms_mqtt_ensure_connected() != 0) {
        LOG_ERR("Attempted to connect to LTE failed");
        return;
    }
    char payload[256];
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
    
    LOG_INF("Publishing LTE Payload: %s", payload);
    comms_mqtt_publish(payload);
}

/* 
    Function that starts a whole localization pipeline.
    Attempts wi-fi scanning first and GNSS and LTE as fallbacks

    Handles required modem activation and deactivation.

    Returns true or false based on localization result.

    USED as the core part of localization_thread_fn
*/
static bool perform_localization_work(void)
{
    int batt = get_battery_level();
    uint32_t events;

    /* Clear ALL stale flags from any previous run, up front */
    k_event_clear(&app_events, WIFI_EVT_SUCCESS | WIFI_EVT_FAIL | LOC_EVT_VERDICT);

    /* STEP 1: WI-FI */
    LOG_INF("Taking MQTT/LTE offline for Wi-Fi Scan.");
    comms_mqtt_disconnect();
    comms_lte_sleep();

    k_sleep(K_MSEC(1000));

    /* Enable LDO1 before bringing up Wi-Fi */
    if (device_is_ready(ldo1_dev)) {
        regulator_enable(ldo1_dev);
    }

    struct net_if *iface = net_if_get_wifi_sta();
    if (iface) net_if_up(iface);

    int aps = do_wifi_scan();
    const struct ap_data_t *best_aps = comms_wifi_get_aps();

    if (iface) net_if_down(iface);
    k_sleep(K_MSEC(500));

    /* Disable LDO1 after bringing down Wi-Fi */
    if (device_is_ready(ldo1_dev)) {
        regulator_disable(ldo1_dev);
    }

    comms_lte_wake();

    if (aps > 0) {
        k_event_clear(&app_events, WIFI_EVT_SUCCESS | WIFI_EVT_FAIL | LOC_EVT_VERDICT);

        send_wifi_payload(batt, aps, best_aps);

        events = k_event_wait(&app_events, WIFI_EVT_SUCCESS | WIFI_EVT_FAIL, false, K_SECONDS(10));

        if (events & WIFI_EVT_SUCCESS) {
            LOG_INF("Server confirmed Wi-Fi. Waiting for Home/Away verdict...");

            events = k_event_wait(&app_events, LOC_EVT_VERDICT, false, K_SECONDS(10));

            if (events & LOC_EVT_HOME) {
                LOG_INF("Server verdict (Wi-Fi): HOME -- inside geofence.");
                return true;
            } else if (events & LOC_EVT_AWAY) {
                LOG_INF("Server verdict (Wi-Fi): AWAY -- outside geofence.");
                return true;
            } else {
                LOG_WRN("Server verdict timeout after Wi-Fi success (10 s).");
                return false;
            }
        } else if (events & WIFI_EVT_FAIL) {
            LOG_INF("Server failed to locate Wi-Fi APs. Moving to GNSS.");
        } else {
            LOG_INF("Server Wi-Fi response timeout. Moving to GNSS.");
        }
    } else {
        LOG_WRN("Wi-Fi scan found no access points. Moving to GNSS.");
    }

    /* STEP 2: GNSS FALLBACK */
    LOG_INF("Attempting GNSS Localization.");
    bool assisted = false;
    if (comms_mqtt_ensure_connected() == 0) {
        assisted = (comms_agnss_refresh_if_needed() == 0);
        LOG_INF("A-GNSS assistance: %s", assisted ? "OK" : "unavailable");
    }
    comms_mqtt_disconnect();
    comms_lte_gnss_mode();
    int gnss_res = do_gnss_fix_timeout(assisted ? 45 : 120);
    if (gnss_res != 0 && assisted) {
        comms_agnss_invalidate();   /* force a refetch next cycle */
    }
    comms_lte_normal_mode();

    k_event_clear(&app_events, LOC_EVT_VERDICT);

    if (gnss_res == 0) {
        LOG_INF("GNSS Fix acquired.");
        send_gnss_payload(batt, comms_gnss_get_pvt());
    } else {
        LOG_INF("GNSS Failed. Falling back to LTE Cell Tower location.");
        send_lte_payload(batt);
    }

    events = k_event_wait(&app_events, LOC_EVT_VERDICT, false, K_SECONDS(10));

    if (events & LOC_EVT_HOME) {
        LOG_INF("Server verdict (%s): HOME -- inside geofence.",
                gnss_res == 0 ? "GNSS" : "LTE cell-ID");
        return true;
    } else if (events & LOC_EVT_AWAY) {
        LOG_INF("Server verdict (%s): AWAY -- outside geofence.",
                gnss_res == 0 ? "GNSS" : "LTE cell-ID");
        return true;
    } else {
        LOG_WRN("No server verdict received within 10s timeout.");
        return false;
    }
}


void comms_send_alert_status(enum alert_reason reason)
{
    current_status = (reason == REASON_FALL_DETECTED) ? "fall" : "panic";
    
    /* Ensure the modem is awake and MQTT is connected to send the alert */
    comms_lte_wake();
    int batt = get_battery_level();
    send_status_payload(batt);
}

/* Routine localization trigger */
int comms_update_localization(void)
{
    if (!atomic_cas(&loc_busy, 0, 1)) {
        LOG_WRN("Localization already in flight -- request rejected");
        return -EBUSY;
    }
    k_sem_give(&loc_start_sem);
    return 0;
}

/* Clear the alert status when exiting the emergency */
void comms_clear_alert(void) {
    current_status = "ok";
}

static void localization_thread_fn(void *arg1, void *arg2, void *arg3)
{
    while (1) {
        k_sem_take(&loc_start_sem, K_FOREVER);
        
        sensors_localization_blink_start(); // Turns on LED yellow blink

        bool success = perform_localization_work();
        
        sensors_localization_blink_stop(); // Turns off LED yellow blink

        atomic_set(&loc_busy, 0);

        /* Broadcast the exact outcome to the FSM */
        struct bracelet_event ev;
        ev.type = success ? EVENT_LOC_SUCCESS : EVENT_LOC_FAILURE;
        zbus_chan_pub(&fsm_events_chan, &ev, K_NO_WAIT);
    }
}

K_THREAD_DEFINE(loc_thread, 4096, localization_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, 0);

/* Initialization function, called in main.c */
int comms_init(void)
{
    comms_wifi_init();
    comms_gnss_init();
    
    if (comms_network_init() != 0) {
        return -1;
    }
    
    LOG_INF("Comms initialization finished.");
    return 0;
}
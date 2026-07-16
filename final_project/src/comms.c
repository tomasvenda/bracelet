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
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_mgmt.h>
#include <nrf_modem_at.h>


#include "events.h"
#include "comms.h"

LOG_MODULE_REGISTER(comms_system, LOG_LEVEL_INF);

/* ------------------------------------------------------------------
 * HARDWARE BINDINGS & STATE
 * ------------------------------------------------------------------ */

static K_SEM_DEFINE(wifi_scan_sem, 0, 1);
static K_SEM_DEFINE(gnss_fix_sem, 0, 1);
static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(mqtt_connected_sem, 0, 1);

K_EVENT_DEFINE(app_events); // event flag used purely for the Wi-Fi localisation handshake

#define WIFI_EVT_SUCCESS BIT(0)
#define WIFI_EVT_FAIL    BIT(1)
#define LOC_EVT_VERDICT  BIT(2)  /* located_home/located_away arrived */
#define LOC_EVT_ABORT    BIT(3)  /* alert preemption: bail out of waits */

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

#define MAX_APS_PER_SCAN 20
static struct ap_data_t { char mac[18]; int8_t rssi; } best_aps[MAX_APS_PER_SCAN];
static struct nrf_modem_gnss_pvt_data_frame last_pvt;
static bool gnss_has_fix = false;

/* Set by comms_send_alert() to preempt a ROUTINE waterfall mid-flight
 * (e.g. during the 180 s GNSS wait). Cleared when a new run starts.
 * Declared up here because do_gnss_fix()/perform_localization_work()
 * reference it before the thread section. */
static atomic_t loc_abort = ATOMIC_INIT(0);
/* True while the in-flight run carries alert status (fall/panic). */
static bool run_is_alert;
static uint32_t current_ap_count;  // Number of AP's that are confirmed and stored
static uint32_t scan_result;    // Number of AP's reported during a scan

/* Dummy battery function - Replace with actual ADC logic later */
static int get_battery_level(void) { return 85; }

/* ------------------------------------------------------------------
 * EVENT HANDLERS (WIFI & GNSS)
 * ------------------------------------------------------------------ */

/*---------------------------------------------------------------
 * Wi-Fi event handler
 *
 * This function is called automatically by Zephyr whenever a
 * registered Wi-Fi management event occurs.
 *--------------------------------------------------------------*/
static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event, struct net_if *iface)
{
    switch (mgmt_event) {

    case NET_EVENT_WIFI_SCAN_RESULT:
    {
        const struct wifi_scan_result *entry =
            (const struct wifi_scan_result *)cb->info;

        scan_result++;

        LOG_INF("[%02d] BSSID=%02X:%02X:%02X:%02X:%02X:%02X  RSSI=%d dBm  CH=%d  SSID=%s",
            scan_result,
            entry->mac[0],
            entry->mac[1],
            entry->mac[2],
            entry->mac[3],
            entry->mac[4],
            entry->mac[5],
            entry->rssi,
            entry->channel,
            entry->ssid);
        
        /* Save the MAC and RSSI for the MQTT payload */
        char mac_string_buf[18];
        snprintf(mac_string_buf, sizeof(mac_string_buf), "%02X:%02X:%02X:%02X:%02X:%02X",
                 entry->mac[0], entry->mac[1], entry->mac[2], 
                 entry->mac[3], entry->mac[4], entry->mac[5]);

        /* Prevent duplicates and store the MAX_APS_PER_SCAN strongest signals */
        bool duplicate = false;
        for (int i = 0; i < current_ap_count; i++) {
            if (strcmp(best_aps[i].mac, mac_string_buf) == 0) duplicate = true;
        }

        if (!duplicate) {
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

        break;
    }

    case NET_EVENT_WIFI_SCAN_DONE:
    {
        const struct wifi_status *status = (const struct wifi_status *)cb->info;

        LOG_INF("Scan done status=%d", status->status);

        k_sem_give(&wifi_scan_sem);
        break;
    }

    default:
        break;
    }
}

static void gnss_event_handler(int event)
{
    if (event == NRF_MODEM_GNSS_EVT_PVT) {
        if (nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT) == 0) {

            /* Print every tracked satellite each PVT (once per second in
             * continuous mode). C/N0 is reported in 0.1 dB-Hz units.
             * Runs in ISR context, so keep it to printk and no blocking. */
            int tracked = 0;
            int used = 0;
            for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
                if (last_pvt.sv[i].sv == 0) {
                    continue; /* empty slot */
                }
                tracked++;
                bool in_fix = last_pvt.sv[i].flags &
                              NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX;
                if (in_fix) {
                    used++;
                }
                printk("  SV %3u  C/N0 %2u.%u dB-Hz  elev %2d  %s\n",
                       last_pvt.sv[i].sv,
                       last_pvt.sv[i].cn0 / 10, last_pvt.sv[i].cn0 % 10,
                       last_pvt.sv[i].elevation,
                       in_fix ? "[in fix]" : "");
            }
            printk("GNSS: tracking %d satellites (%d used in fix)\n",
                   tracked, used);

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

 /* Registration-denied backoff, WITHOUT sleeping inside the LTE event
 * handler (that blocked all subsequent LTE events for 2 minutes). */
static void lte_backoff_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(lte_backoff_work, lte_backoff_fn);
static bool in_backoff;

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
    
    if (strcmp(buf, "wifi_successful") == 0) {
        k_event_post(&app_events, WIFI_EVT_SUCCESS);
        return;
    }
    else if (strcmp(buf, "wifi_failed") == 0) {
        k_event_post(&app_events, WIFI_EVT_FAIL);
        return;
    }

    /* 2. ZBUS FSM Events (if the payload is JSON) */
    struct bracelet_event event;
    if (strstr(buf, "located_home")) {
        event.type = EVENT_SERVER_REPLY_HOME;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
        k_event_post(&app_events, LOC_EVT_VERDICT);
    } 
    else if (strstr(buf, "located_away")) {
        event.type = EVENT_SERVER_REPLY_AWAY;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
        k_event_post(&app_events, LOC_EVT_VERDICT);
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
    
    /* 1. WAKE THE MODEM UP! (Brings it out of lte_lc_power_off state) */
    lte_lc_normal();
    
    /* 2. Wait for LTE registration */
    enum lte_lc_nw_reg_status reg_status;
    lte_lc_nw_reg_status_get(&reg_status);
    
    if (reg_status != LTE_LC_NW_REG_REGISTERED_HOME && reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING) {
        LOG_INF("Waiting for LTE...");
        k_sem_reset(&lte_connected);
        if (k_sem_take(&lte_connected, K_SECONDS(120)) != 0) {
            LOG_ERR("LTE Reconnect failed!");
            return -ENETUNREACH;
        }
    }

    struct mqtt_helper_conn_params conn_params = {
        .hostname.ptr = CONFIG_MQTT_BROKER_HOSTNAME,
        .hostname.size = strlen(CONFIG_MQTT_BROKER_HOSTNAME),
        .device_id.ptr = CLIENT_ID,
        .device_id.size = strlen(CLIENT_ID),
    };
    
    if (mqtt_helper_connect(&conn_params) != 0) return -EIO;

    if (k_sem_take(&mqtt_connected_sem, K_SECONDS(15)) != 0) return -ETIMEDOUT;

    /* CLEAN_SESSION=y: broker forgot our subscriptions -- restore them */
    int err = mqtt_helper_subscribe(&sub_list);
    if (err) {
        LOG_ERR("Re-subscribe failed: %d", err);
        return err;
    }
    LOG_INF("Re-subscribed to %s", MQTT_SUB_TOPIC);
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

/* WIFI STUFF */

/* Callback object used by the Zephyr network manager to report Wi-Fi events. */
static struct net_mgmt_event_callback wifi_cb;

int do_wifi_scan(void){
    /* Fetch the interface. Zephyr Auto-Init created this during boot. */
    struct net_if *iface = net_if_get_wifi_sta();

    if (!iface) {
        LOG_ERR("No Wi-Fi interface found");
        return -1;
    }
    LOG_INF("[OK]   Wi-Fi net_if found (index=%d).", net_if_get_by_iface(iface));

    /* FIX: callback is already registered once in comms_init().
     * Re-adding the same net_mgmt_event_callback node corrupts the
     * event callback list (same slist node inserted twice). */

    struct wifi_scan_params params = {0};

    /* Active scan sends probe requests instead of only listening. */
    params.scan_type = WIFI_SCAN_TYPE_ACTIVE;

    /* Scan both 2.4 GHz and 5 GHz. */
    params.bands =
        BIT(WIFI_FREQ_BAND_2_4_GHZ) |
        BIT(WIFI_FREQ_BAND_5_GHZ);

    /* Stay longer on each channel. */
    params.dwell_time_active = 120;
    params.dwell_time_passive = 120;

    /* Return as many APs as possible. */
    params.max_bss_cnt = 64;

    scan_result = 0;
    current_ap_count = 0;
    k_sem_reset(&wifi_scan_sem); /* drop any stale/abort-time token */

    // Sends the scan request to the driver
    int ret = net_mgmt(
        NET_REQUEST_WIFI_SCAN,
        iface,
        &params,
        sizeof(params));

    if (ret) {
        LOG_ERR("Failed to start scan (%d)", ret);
        return ret;
    }

    LOG_INF("Scanning...");

    k_sem_take(&wifi_scan_sem, K_SECONDS(20));

    LOG_INF("Found %d access points", current_ap_count);

    return current_ap_count;
}

static int do_gnss_fix(void)
{
    gnss_has_fix = false;
    
    /* VERY IMPORTANT: Clear the semaphore in case of stale events 
     * before we begin waiting. */
    k_sem_reset(&gnss_fix_sem); 

    if (mqtt_is_connected) {
        mqtt_helper_disconnect();
        k_msleep(300);
        mqtt_is_connected = false;
    }

    /* FIX: GNSS only gets RF time when LTE is idle. Without PSM/eDRX,
     * an RRC-connected LTE link (we just published over MQTT) starves
     * GNSS completely. Take LTE down for the duration of the fix. */
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
    
    nrf_modem_gnss_fix_interval_set(0);    /* Single fix mode */
    nrf_modem_gnss_fix_retry_set(180);     /* 180s hardware timeout */
    nrf_modem_gnss_start();

    LOG_INF("Searching for GNSS satellites (up to 3 minutes)...");
    
    /* Wait for the event handler to give the semaphore upon a valid fix */
    int res = k_sem_take(&gnss_fix_sem, K_SECONDS(180));
    
    nrf_modem_gnss_stop();
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_GNSS);

    /* On abort, leave LTE DOWN: if this is an ACK-abort the FSM is
     * powering the modem off for deep sleep right now, and racing it
     * with ACTIVATE_LTE leaves the modem on while the device sleeps.
     * The next waterfall's mqtt_ensure_connected() wakes LTE itself. */
    if (atomic_get(&loc_abort)) {
        LOG_WRN("GNSS search aborted.");
        return -ECANCELED;
    }

    /* Restore LTE so the payload can be published afterwards. */
    lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_LTE);
    
    /* Sem can also be given by an abort race; trust the fix flag. */
    return (res == 0 && gnss_has_fix) ? 0 : -ETIMEDOUT;
}

/* ------------------------------------------------------------------
 * BACKGROUND WORKER: LOCALIZATION WATERFALL
 * ------------------------------------------------------------------ */
static void perform_localization_work(void)
{
    static char payload[2048]; 
    int offset = 0;
    int batt = get_battery_level();
    int aps = 0;

    /* This run owns the abort flag from here on. */
    atomic_clear(&loc_abort);
    k_event_clear(&app_events, LOC_EVT_ABORT);

    /* STEP 0: ALERT runs inform the server FIRST, before any
     * localization. The status (fall/panic) is the life-critical
     * part; the position refines it afterwards. */
    if (strcmp(current_status, "ok") != 0) {
        LOG_WRN("ALERT run: notifying server before localization.");
        snprintf(payload, sizeof(payload),
            "{\"status\":\"%s\", \"battery\":%d}", current_status, batt);
        if (lte_mqtt_publish_str(payload) != 0) {
            LOG_ERR("Immediate alert notify failed; continuing to localize.");
        }
    }

    /* STEP 1: WI-FI */
    
    /* Cleanly drop MQTT BEFORE killing its transport, so
     * mqtt_ensure_connected() does a real reconnect afterwards. */
    if (mqtt_is_connected) {
        mqtt_helper_disconnect();
        k_msleep(300);
        mqtt_is_connected = false;
    }
    LOG_INF("Taking LTE offline for Wi-Fi Scan.");
    lte_lc_offline();
    k_sleep(K_MSEC(500));
    
    k_sleep(K_MSEC(1000)); // Allow PMIC rail to stabilize

    /* --- WAKE THE NRF7002 --- */
    struct net_if *iface = net_if_get_wifi_sta();
    if (iface) {
        net_if_up(iface);
        k_sleep(K_MSEC(100)); /* Let the driver boot the chip */
    }

    // Calling the actual wifi scan function
    aps = do_wifi_scan();

    if (iface) {
        net_if_down(iface); /* Drops bucken-gpios LOW automatically */
    }
    k_sleep(K_MSEC(500));
    
    /* Abort check BEFORE re-activating LTE: on an ACK-abort the FSM
     * is shutting the modem down for deep sleep; on an alert-abort
     * the alert run wakes LTE itself. Either way, leave it down. */
    if (atomic_get(&loc_abort)) {
        LOG_WRN("Localization run aborted.");
        return;
    }

    lte_lc_normal();
    // sketchy
    /* Return to normal cellular operation */
    //lte_lc_connect_async(lte_handler);    

    if (aps > 0) {
        /* payload[2048] safely holds 20+ APs (~58 B each + header) */
        offset = 0;
        int remaining = sizeof(payload);
        int written = 0;

        /* Build the header */
        written = snprintf(payload + offset, remaining,
            "{\"status\":\"%s\", \"battery\":%d, \"wifi\":{\"accessPoints\":[", current_status, batt);
        offset += written;
        remaining -= written;

        for (int i = 0; i < aps; i++) {
            /* Worst-case AP entry is ~58 bytes; keep headroom for it
             * plus the closing brackets so JSON is never truncated
             * mid-object (which would also desync offset/remaining). */
            if (remaining < 64) {
                LOG_WRN("Payload buffer full! Truncating Wi-Fi list at %d APs.", i);
                break; 
            }

            written = snprintf(payload + offset, remaining,
                "{\"macAddress\":\"%s\",\"signalStrength\":%d}%s",
                best_aps[i].mac, best_aps[i].rssi, (i < aps - 1) ? "," : "");
            
            offset += written;
            remaining -= written;
        }

        /* Build the footer */
        snprintf(payload + offset, remaining, "]}}");

        LOG_INF("Publishing Wi-Fi Payload (%d bytes)", offset);
        lte_mqtt_publish_str(payload);

        k_event_clear(&app_events, WIFI_EVT_SUCCESS | WIFI_EVT_FAIL);
        uint32_t events = k_event_wait(&app_events,
                                       WIFI_EVT_SUCCESS | WIFI_EVT_FAIL |
                                       LOC_EVT_ABORT,
                                       false, K_SECONDS(10));

        if ((events & LOC_EVT_ABORT) || atomic_get(&loc_abort)) {
            LOG_WRN("Routine localization aborted by alert.");
            return;
        }

        if (events & WIFI_EVT_SUCCESS) {
            LOG_INF("Server confirmed Wi-Fi localization -> done.");
            return;
        } else if (events & WIFI_EVT_FAIL) {
            LOG_INF("Server rejected Wi-Fi data.");
        } else {
            LOG_INF("Server response timeout (10s).");
        }
    } else {
        LOG_WRN("Wi-Fi scan found no access points.");
    }

#if defined(CONFIG_LOC_WIFI_ONLY)
    /* Fallbacks intentionally disabled during bring-up. The STATUS
     * must still reach the server even without a location -- critical
     * for fall/panic alerts. (The Wi-Fi payload above already carried
     * the status if aps > 0, but publish a bare one if it didn't.) */
    LOG_WRN("Wi-Fi localization inconclusive; GNSS/LTE fallback disabled (LOC_WIFI_ONLY).");
    if (aps == 0) {
        snprintf(payload, sizeof(payload),
            "{\"status\":\"%s\", \"battery\":%d}", current_status, batt);
        LOG_INF("Publishing bare status payload: %s", payload);
        lte_mqtt_publish_str(payload);
    }
    return;
#else
    /* ===== FALLBACK WATERFALL (enabled when LOC_WIFI_ONLY=n) ===== */
    LOG_INF("Wi-Fi failed or inconclusive. Attempting GNSS Localization.");

    int gnss_res = do_gnss_fix();
    if (gnss_res == -ECANCELED) {
        /* Aborted by alert: LTE already restored inside do_gnss_fix,
         * and the alert waterfall is queued. Leave immediately. */
        return;
    }
    if (gnss_res == 0) {
        LOG_INF("GNSS Fix acquired.");
        snprintf(payload, sizeof(payload),
            "{\"status\":\"%s\", \"battery\":%d, \"gnss\":{\"lat\":%.6f, \"lon\":%.6f, \"accuracy\":%.1f}}",
            current_status, batt, last_pvt.latitude, last_pvt.longitude, (double)last_pvt.accuracy);
    }
    else {
        LOG_INF("GNSS Timeout, attempting LTE...");
        /* FIX: if LTE is already registered, no new NW_REG event will
         * arrive and waiting on the semaphore times out after 60 s even
         * though cell info is available right now. Check status first. */
        enum lte_lc_nw_reg_status reg;
        bool lte_ready = (lte_lc_nw_reg_status_get(&reg) == 0) &&
                         (reg == LTE_LC_NW_REG_REGISTERED_HOME ||
                          reg == LTE_LC_NW_REG_REGISTERED_ROAMING);
        if (!lte_ready) {
            k_sem_reset(&lte_connected);
            lte_ready = (k_sem_take(&lte_connected, K_SECONDS(60)) == 0);
        }
        if (lte_ready) {
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

    LOG_INF("Publishing Payload: %s", payload);
    k_event_clear(&app_events, LOC_EVT_VERDICT);
    if (lte_mqtt_publish_str(payload) != 0) {
        LOG_ERR("Failed to publish! (TODO: Save to SPI Flash NVS here)");
    } else {
        /* The Wi-Fi path waits for a verdict; this path must too,
         * otherwise the next waterfall tears MQTT down ~300 ms after
         * publish and the server's located_home/away reply hits a
         * dead socket (verdict lost even though the server resolved). */
        uint32_t ev = k_event_wait(&app_events,
                                   LOC_EVT_VERDICT | LOC_EVT_ABORT,
                                   false, K_SECONDS(10));
        if (ev & LOC_EVT_ABORT) {
            LOG_WRN("Verdict wait aborted.");
        } else if (ev & LOC_EVT_VERDICT) {
            LOG_INF("Server verdict received for GNSS/LTE payload.");
        } else {
            LOG_WRN("No server verdict within 10 s of GNSS/LTE payload.");
        }
    }
#endif /* CONFIG_LOC_WIFI_ONLY */
}


/* ==================================================================
 * DEDICATED THREAD: LOCALIZATION
 * ================================================================== */
K_SEM_DEFINE(loc_start_sem, 0, 1);

/* Set while a localization waterfall is running. Periodic tracking
 * pings are skipped when busy (they'd only queue an immediate
 * back-to-back rerun); alerts always queue regardless. */
static atomic_t loc_busy = ATOMIC_INIT(0);

static void localization_thread_fn(void *arg1, void *arg2, void *arg3)
{
    while (1) {
        /* Wait here peacefully until a button press or ping triggers us */
        k_sem_take(&loc_start_sem, K_FOREVER);
        
        /* Run the waterfall without blocking Zephyr! */
        run_is_alert = (strcmp(current_status, "ok") != 0);
        atomic_set(&loc_busy, 1);
        perform_localization_work();
        atomic_set(&loc_busy, 0);

        /* An ALERT stack that ran to completion (any outcome: wifi
         * verdict, gnss fix, lte fallback, or nothing) tells the FSM,
         * which then moves to ACTIVE_TRACKING. Not sent if the run
         * was aborted (only routine runs can be aborted now). */
        if (run_is_alert && !atomic_get(&loc_abort)) {
            struct bracelet_event ev = { .type = EVENT_LOC_DONE };
            zbus_chan_pub(&fsm_events_chan, &ev, K_NO_WAIT);
        }
    }
}

/* Create a standalone thread with a 4KB stack so it never blocks the OS */
K_THREAD_DEFINE(loc_thread, 4096, localization_thread_fn, NULL, NULL, NULL, K_PRIO_PREEMPT(7), 0, 0);

/* ------------------------------------------------------------------
 * PUBLIC FSM COMMANDS
 * ------------------------------------------------------------------ */
void comms_update_localization(void)
{
    if (atomic_get(&loc_busy)) {
        LOG_INF("Localization already in progress; skipping this ping.");
        return;
    }
    current_status = "ok";
    k_sem_give(&loc_start_sem); /* Wake the thread */
}

// Clears the current status from alert. Called on server ACK.
void comms_clear_alert(void) {
    /* The server ACK means "panic message received" -- NOT "emergency
     * over". It must never abort the localization stack; the alert
     * run always completes wifi -> gnss -> lte and reports LOC_DONE.
     * Status flips back to "ok" with the next routine ping. */
    current_status = "ok";
}

void comms_send_alert(enum alert_reason reason)
{
    current_status = (reason == REASON_FALL_DETECTED) ? "fall" : "panic";

    if (atomic_get(&loc_busy)) {
        if (run_is_alert) {
            /* An alert waterfall is already in flight and has already
             * notified the server. Let it finish; do NOT abort it or
             * the 60 s re-send timer would kill its own GNSS attempt
             * every cycle. A fresh run starts once it completes. */
            LOG_WRN("Alert waterfall already in flight; not restarting it.");
            k_sem_give(&loc_start_sem);
            return;
        }
        /* ROUTINE run in flight: ABORT IT. Unblock every wait it
         * could be sitting in so it falls through immediately. */
        LOG_WRN("ALERT! Aborting in-flight routine localization.");
        atomic_set(&loc_abort, 1);
        k_sem_give(&gnss_fix_sem);              /* break 180 s GNSS wait */
        k_sem_give(&wifi_scan_sem);             /* break 20 s scan wait  */
        k_event_post(&app_events, LOC_EVT_ABORT); /* break verdict waits */
    }

    k_sem_give(&loc_start_sem); /* Queue the alert waterfall */
}

/* ------------------------------------------------------------------
 * INITIALIZATION
 * ------------------------------------------------------------------ */
int comms_init(void)
{
    int err;

    /* Register callbacks for wifi scan results and scan completion.*/
    net_mgmt_init_event_callback(
        &wifi_cb,
        wifi_event_handler,
        NET_EVENT_WIFI_SCAN_RESULT |
        NET_EVENT_WIFI_SCAN_DONE);
    
    net_mgmt_add_event_callback(&wifi_cb);

    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem init failed, err %d", err);
        return -1;
    }

    /* Register callback for gnss.
     * FIX: must be done AFTER nrf_modem_lib_init() -- calling any
     * nrf_modem_gnss_* API before modem init fails, and the error
     * was not checked, so the handler was never registered. */
    err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
    if (err) {
        LOG_ERR("Failed to set GNSS event handler: %d", err);
        return -1;
    }

    /* ==========================================================
     * NEW: Enable COEX0 for the external GNSS LNA
     * Must be sent here, before LTE is activated!
     * ========================================================== */
    err = nrf_modem_at_printf("AT%%XCOEX0=1,1,1565,1586");
    if (err) {
        LOG_ERR("Failed to configure COEX0 for LNA: %d", err);
    } else {
        LOG_INF("COEX0 configured: External LNA enabled for GNSS");
    }

    /* Connecting to LTE */
    LOG_INF("Connecting to LTE network... (This may take a few seconds)");
    err = lte_lc_connect_async(lte_handler);
    if (err) {
        LOG_ERR("LTE connect async failed, err %d", err);
        return -1;
    }

    /* Wait for LTE to connect */
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
    err = mqtt_helper_subscribe(&sub_list);
    if (err) {
        LOG_ERR("Failed to subscribe to MQTT topic: %d", err);
        return err;
    }
    LOG_INF("Subscribed to %s...", MQTT_SUB_TOPIC);

    LOG_INF("Comms initialization finished.");

    return 0;
}

void comms_safe_disconnect(void)
{
    LOG_WRN("=== SAFE DISCONNECT TRIGGERED ===");
    
    if (mqtt_is_connected) {
        LOG_INF("Disconnecting MQTT...");
        mqtt_helper_disconnect();
        k_msleep(1000); /* Give TCP time to actually close the socket */
        mqtt_is_connected = false;
    }

    LOG_INF("Gracefully detaching from LTE network...");
    lte_lc_power_off(); /* Sends the Detach Request to the cell tower */
    
    /* Reset the semaphore so mqtt_ensure_connected waits next time */
    k_sem_reset(&lte_connected); 
    
    LOG_INF("=== SAFE TO POWER OFF OR FLASH NEW CODE ===");
}
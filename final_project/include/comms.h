#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>
#include <stddef.h> 
#include <zephyr/sys/util.h>

#define WIFI_EVT_SUCCESS BIT(0)
#define WIFI_EVT_FAIL    BIT(1)
#define LOC_EVT_HOME     BIT(3)
#define LOC_EVT_AWAY     BIT(4)
#define LOC_EVT_VERDICT  (LOC_EVT_HOME | LOC_EVT_AWAY)

extern struct k_event app_events;

/* Trigger reasons mapped to your Python backend statuses */
enum alert_reason {
    REASON_BUTTON_PRESSED, /* Maps to "panic" */
    REASON_FALL_DETECTED   /* Maps to "fall" */
};

struct nrf_modem_gnss_pvt_data_frame;

// WIFI APS
struct ap_data_t { 
    char mac[18]; 
    int8_t rssi; 
};

/* Wi-Fi Subsystem Prototypes */
void comms_wifi_init(void);
int do_wifi_scan(void);
const struct ap_data_t* comms_wifi_get_aps(void); 

/* GNSS Subsystem Prototypes */
int comms_gnss_init(void);
int do_gnss_fix(void);
int do_gnss_fix_timeout(int timeout_s);
const struct nrf_modem_gnss_pvt_data_frame* comms_gnss_get_pvt(void);

/* A-GNSS*/
typedef void (*comms_raw_cb_t)(const uint8_t *data, size_t len);
void comms_set_raw_response_cb(comms_raw_cb_t cb);
int  comms_agnss_refresh_if_needed(void);
void comms_agnss_invalidate(void);


/* Network (LTE & MQTT) */
int comms_network_init(void);
int comms_mqtt_ensure_connected(void);
int comms_mqtt_publish(const char *payload);
void comms_mqtt_disconnect(void);


/* LTE Controls for Core */
void comms_lte_wake(void);
void comms_lte_sleep(void);
void comms_lte_gnss_mode(void);
void comms_lte_normal_mode(void);
void comms_safe_disconnect(void);

/* Core APIs (Used by fsm.c) */
int comms_init(void);
void comms_send_alert_status(enum alert_reason reason);
int comms_update_localization(void);
void comms_clear_alert(void);


#endif /* COMMS_H */
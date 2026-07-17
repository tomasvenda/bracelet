#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>
#include <zephyr/sys/util.h>

#define WIFI_EVT_SUCCESS BIT(0)
#define WIFI_EVT_FAIL    BIT(1)
#define LOC_EVT_VERDICT  BIT(2)

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
const struct nrf_modem_gnss_pvt_data_frame* comms_gnss_get_pvt(void);

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
void comms_update_localization(void);
void comms_clear_alert(void);


#endif /* COMMS_H */
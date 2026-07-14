#ifndef EVENTS_H
#define EVENTS_H

#include <zephyr/zbus/zbus.h>

/* These are all the physical and network triggers defined in FSM scheme */
enum bracelet_event_type {
    EVENT_IMU_LIGHT_MOTION, /* Like walking*/
    EVENT_IMU_HARSH_IMPACT, /* A fall */
    EVENT_BUTTON_PRESSED,
    EVENT_SERVER_REPLY_HOME, /* INSIDE GEOFENCE */
    EVENT_SERVER_REPLY_AWAY,      /* OUTSIDE GEOFENCE */
    EVENT_SERVER_ACK_ALERT, /* The server acknowledges the alert */
    EVENT_ML_FALL_DETECTED, /* TinyML output */
    EVENT_ML_NO_FALL, /* TinyML output */
    EVENT_TIMER_1MIN_EXPIRED,
    EVENT_TIMER_10MIN_EXPIRED,
    EVENT_SERVER_WIFI_SUCCESS,    // WiFi scanning success on server side
    EVENT_SERVER_WIFI_FAIL,     // WiFi scanning failed on server side  
    EVENT_STATE_TIMEOUT,     /* Per-state watchdog (LOCATION_PING / ALERT) */
};

/* The ZBUS message payload */
struct bracelet_event {
    enum bracelet_event_type type;
};

/* Declare the ZBUS channel so all files can see it */
ZBUS_CHAN_DECLARE(fsm_events_chan);

#endif /* EVENTS_H */
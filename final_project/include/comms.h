#ifndef COMMS_H
#define COMMS_H

#include <stdint.h>

/* Trigger reasons mapped to your Python backend statuses */
enum alert_reason {
    REASON_BUTTON_PRESSED, /* Maps to "panic" */
    REASON_FALL_DETECTED   /* Maps to "fall" */
};

/* Initializes Modem, LTE, MQTT, and the Wi-Fi PMIC regulator */
int comms_init(void);

/* Called by FSM during LOCATION_PING and ACTIVE_TRACKING. 
 * Triggers the Wi-Fi -> GNSS -> CellID waterfall and sends status "ok". */
void comms_update_localization(void);

/* Called by FSM during ALERT state. 
 * Sends the emergency status ("fall" or "panic") and forces a location update. */
void comms_send_alert(enum alert_reason reason);

// Clear the panic/fall status from the variable current_status
void comms_clear_alert(void); 

#endif /* COMMS_H */
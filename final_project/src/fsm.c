#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>
#include <stdio.h>
#include "events.h"
#include "sensors.h"


/* Define the ZBUS channel (Message size, subscribers, etc.) */
ZBUS_CHAN_DEFINE(fsm_events_chan,       /* Name */
                 struct bracelet_event, /* Payload type */
                 NULL,                  /* Validator */
                 NULL,                  /* User data */
                 ZBUS_OBSERVERS(fsm_sub), /* Initial observers */
                 ZBUS_MSG_INIT(0));     /* Initial value */

/* Define the ZBUS subscriber for the FSM thread */
// We set a queue size of 4 if the events come faster than it can process
ZBUS_SUBSCRIBER_DEFINE(fsm_sub, 4);

/* Forward declaration of the state table */
static const struct smf_state states[];

/* Enumerate states */
enum state_names {
    STATE_DEEP_SLEEP,
    STATE_LOCATION_PING,
    STATE_ACTIVE_TRACKING,
    STATE_EVALUATION,
    STATE_ALERT
};

/* The FSM Context: Holds the current state and memory of the previous state */
struct fsm_context {
    struct smf_ctx smf;
    enum state_names previous_state;
    struct bracelet_event current_event;
} fsm;

/// TODO: Replace with actual TinyML header when inference model is integrated 
static inline int run_fall_inference(float *buf) { (void)buf; return 0; }

/* ======================================================================
 * STATE ACTIONS (Entry & Run logic for each bubble in the flowchart)
 * ====================================================================== */

// --- DEEP SLEEP ---
static void deep_sleep_entry(void *o) {
    printf("[STATE] Entered DEEP_SLEEP. MCU sleeping. LTE/GNSS off.\n");
    fsm.previous_state = STATE_DEEP_SLEEP; // Update tracker
}

static enum smf_state_result deep_sleep_run(void *o) {
    if (fsm.current_event.type == EVENT_IMU_LIGHT_MOTION) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_LOCATION_PING]);
    }
    return SMF_EVENT_HANDLED;
}


// --- LOCATION PING ---
static void location_ping_entry(void *o) {
    printf("[STATE] Entered LOCATION_PING. Scanning Wi-Fi/GNSS...\n");
    fsm.previous_state = STATE_LOCATION_PING;
    /// TODO: Call comms.c -> update_localization()
    // 
}

static enum smf_state_result location_ping_run(void *o) {
    if (fsm.current_event.type == EVENT_SERVER_REPLY_STATIONARY) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    } else if (fsm.current_event.type == EVENT_SERVER_REPLY_MOVED) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
    }
}

// --- ACTIVE TRACKING ---
static void active_tracking_entry(void *o) {
    printf("[STATE] Entered ACTIVE_TRACKING. High alert tracking.\n");
    fsm.previous_state = STATE_ACTIVE_TRACKING;
    /// TODO: Start the 3-minute Zephyr kernel timer here
}

static enum smf_state_result active_tracking_run(void *o) {
    if (fsm.current_event.type == EVENT_TIMER_3MIN_EXPIRED) {
        printf("        -> 3 Min Timer Expired. Pinging server...\n");
        /// TODO: Call comms.c -> update_localization()
    } else if (fsm.current_event.type == EVENT_SERVER_REPLY_STATIONARY) {
        // Assuming IMU logic is handled before publishing this event
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    }
}

static void do_ml_evaluation_work(struct k_work *work) {
    printf("[EVALUATION] Harsh impact detected! Gathering 2 seconds of sensor data...\n");
    
    // 1. Array to hold the time-series data for the ML model
    float ml_data_buffer[300]; // Example: 50Hz * 2 sec * 3 channels
    
    // 2. Poll BOTH the IMU and Barometer to fill the buffer
    /// TODO: write the right function here
    sensors_collect_ml_window(ml_data_buffer, 300); // We will write this in sensors.c
    
    // 3. Pass the combined buffer to your C++ TinyML wrapper
    int result = run_fall_inference(ml_data_buffer);
    
    // 4. Publish the result back to the FSM ZBUS
    struct bracelet_event event;
    if (result == 1) {
        event.type = EVENT_ML_FALL_DETECTED;
    } else {
        event.type = EVENT_ML_NO_FALL;
    }
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

K_WORK_DEFINE(ml_eval_work, do_ml_evaluation_work);

static void evaluation_entry(void *o) {
    /* Toss the heavy data-collection and ML math to the background worker 
     * so the FSM is free to instantly listen for the Emergency Button! */
    k_work_submit(&ml_eval_work);
}

static enum smf_state_result evaluation_run(void *o) {
    if (fsm.current_event.type == EVENT_ML_NO_FALL) {
        printf("        -> TinyML says False Alarm. Returning to previous state.\n");
        smf_set_state(SMF_CTX(&fsm), &states[fsm.previous_state]);
    } else if (fsm.current_event.type == EVENT_ML_FALL_DETECTED) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ALERT]);
    }
}



// --- ALERT (Emergency) ---
static void alert_entry(void *o) {
    printf("[STATE] Entered ALERT! Sending emergency payload to server!\n");
    fsm.previous_state = STATE_ALERT;
    // TODO: Call comms.c -> update_status(REASON_EMERGENCY)
}

static enum smf_state_result alert_run(void *o) {
    if (fsm.current_event.type == EVENT_SERVER_ACK_ALERT) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    }
}



/* ======================================================================
 * STATE TABLE MAPPING
 * ====================================================================== */


// smf_create_state parameters: (entry, run, exit, parent, custom_data)
static const struct smf_state states[] = {
    [STATE_DEEP_SLEEP]      = SMF_CREATE_STATE(deep_sleep_entry, deep_sleep_run, NULL, NULL, NULL),
    [STATE_LOCATION_PING]   = SMF_CREATE_STATE(location_ping_entry, location_ping_run, NULL, NULL, NULL),
    [STATE_ACTIVE_TRACKING] = SMF_CREATE_STATE(active_tracking_entry, active_tracking_run, NULL, NULL, NULL),
    [STATE_EVALUATION]      = SMF_CREATE_STATE(evaluation_entry, evaluation_run, NULL, NULL, NULL),
    [STATE_ALERT]           = SMF_CREATE_STATE(alert_entry, alert_run, NULL, NULL, NULL),
};

/* ======================================================================
 * MAIN FSM THREAD (Listens to ZBUS)
 * ====================================================================== */
void fsm_thread_main(void)
{
    const struct zbus_channel *chan;
    
    /* Initialize the state machine into Deep Sleep */
    smf_set_initial(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);

    while (1) {
        /* Wait indefinitely for a new event to arrive on ZBUS */
        if (zbus_sub_wait(&fsm_sub, &chan, K_FOREVER) == 0) {
            
            /* Read the payload */
            zbus_chan_read(chan, &fsm.current_event, K_NO_WAIT);

            /* GLOBAL OVERRIDES: Button and Harsh Impact bypass normal state logic */
            if (fsm.current_event.type == EVENT_BUTTON_PRESSED) {
                smf_set_state(SMF_CTX(&fsm), &states[STATE_ALERT]);
                continue;
            } 
            else if (fsm.current_event.type == EVENT_IMU_HARSH_IMPACT) {
                smf_set_state(SMF_CTX(&fsm), &states[STATE_EVALUATION]);
                continue;
            }

            /* Otherwise, pass the event to the current state's 'run' function */
            smf_run_state(SMF_CTX(&fsm));
        }
    }
}

/* Define a Zephyr Thread specifically for the FSM to run in the background */
// K_thread_define parameters: (name, stack_size, entry_function, arg1, arg2, arg3, priority, options, delay on boot)
K_THREAD_DEFINE(fsm_tid, 1024, fsm_thread_main, NULL, NULL, NULL, 5, 0, 0);
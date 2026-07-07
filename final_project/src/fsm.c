#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>
#include <stdio.h>
#include "events.h"
#include "sensors.h"
#include "comms.h"


/* Define the ZBUS channel (Message size, subscribers, etc.) */
ZBUS_CHAN_DEFINE(fsm_events_chan,       /* Name */
                 struct bracelet_event, /* Payload type */
                 NULL,                  /* Validator */
                 NULL,                  /* User data */
                 ZBUS_OBSERVERS(fsm_sub), /* Initial observers */
                 ZBUS_MSG_INIT(0));     /* Initial value */

/* Define the ZBUS subscriber for the FSM thread */
// We set a queue size of 4 if the events come faster than it can process
ZBUS_MSG_SUBSCRIBER_DEFINE(fsm_sub);

/*===========================*/
/* Timer for active tracking */
/*===========================*/
static void tracking_timer_expiry_cb(struct k_timer *timer_id);

// Define the kernel timer
K_TIMER_DEFINE(tracking_timer, tracking_timer_expiry_cb, NULL);

// Timer expiry callback
static void tracking_timer_expiry_cb(struct k_timer *timer_id) {
    struct bracelet_event event = {
        .type = EVENT_TIMER_1MIN_EXPIRED
    };
    
    // Publish to the FSM safely from the timer's interrupt context
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

/*===========================*/
/* Timer for deep sleep      */
/*===========================*/
static void cooldown_timer_expiry_cb(struct k_timer *timer_id);

// Define the kernel timer
K_TIMER_DEFINE(cooldown_timer, cooldown_timer_expiry_cb, NULL);

// Timer expiry callback
static void cooldown_timer_expiry_cb(struct k_timer *timer_id) {
    struct bracelet_event event = {
        .type = EVENT_TIMER_10MIN_EXPIRED
    };
    
    // Publish safely from interrupt context
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

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

    // I am using a static flag to track the very first time this function runs
    static bool is_first_run = true;

    if (is_first_run) {
        printf("[DEEP SLEEP] First boot detected. Enabling IMU wake immediately.\n");
        // Enable motion interrupt right away so the device is ready
        sensors_enable_motion_trigger();
        
        // Ensure this block never runs again until the device reboots
        is_first_run = false; 
    } else {
        printf("[DEEP SLEEP] Re-entering sleep. Starting 10-minute cooldown timer.\n");
        // we disable motion interrupt and start the timer
        sensors_disable_motion_trigger();
        k_timer_start(&cooldown_timer, K_MINUTES(10), K_NO_WAIT);
    }
}

static void deep_sleep_exit(void *o) {
    // Safely stop the timer in case a global override (panic button/fall) 
    // forces an exit before the 10 minutes expire.
    k_timer_stop(&cooldown_timer);
}

static enum smf_state_result deep_sleep_run(void *o) {
    if (fsm.current_event.type == EVENT_TIMER_10MIN_EXPIRED) {
        printf("[DEEP SLEEP] 10 min cooldown passed. Enabling IMU wake interrupt.\n");
        // Re-enable the hardware interrupt so the next movement wakes the nRF9151
        sensors_enable_motion_trigger();
    }
    else if (fsm.current_event.type == EVENT_IMU_LIGHT_MOTION) {
        // This will only ever trigger if the cooldown has passed 
        // and the hardware interrupt was re-enabled.
        smf_set_state(SMF_CTX(&fsm), &states[STATE_LOCATION_PING]);
    }
    return SMF_EVENT_HANDLED;
}

// --- LOCATION PING ---
static void location_ping_entry(void *o) {
    printf("[STATE] Entered LOCATION_PING. Updating location...\n");
    fsm.previous_state = STATE_LOCATION_PING;
    
    // Disable the motion interrupt to save battery while we ping
    sensors_disable_motion_trigger();
    // Performs one time the localization work
    comms_update_localization();
}

static enum smf_state_result location_ping_run(void *o) {
    printf("[STATE] location_ping_run");

    if (fsm.current_event.type == EVENT_SERVER_REPLY_HOME) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    } else if (fsm.current_event.type == EVENT_SERVER_REPLY_AWAY) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
    }

    return SMF_EVENT_HANDLED;
}

// --- ACTIVE TRACKING ---
static void active_tracking_entry(void *o) {
    printf("[STATE] Entered ACTIVE_TRACKING. High alert tracking.\n");
    fsm.previous_state = STATE_ACTIVE_TRACKING;

    // Start the timer: First expiry in 1 min, periodically every 1 mins after
    k_timer_start(&tracking_timer, K_MINUTES(1), K_MINUTES(1));
}

static void active_tracking_exit(void *o) {
    printf("[STATE] Exiting ACTIVE_TRACKING. Stopping timer.\n");
    // Stops the timer safely, even if a PANIC or FALL forces the state change
    k_timer_stop(&tracking_timer);
}

static enum smf_state_result active_tracking_run(void *o) {
    if (fsm.current_event.type == EVENT_TIMER_1MIN_EXPIRED) {
        printf("[ACTIVE TRACKING] 1 Min Timer Expired. Updating location...\n");
        comms_update_localization();
    } else if (fsm.current_event.type == EVENT_SERVER_REPLY_HOME) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    }
    
    return SMF_EVENT_HANDLED;
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

    return SMF_EVENT_HANDLED;
}


// --- ALERT (Emergency) ---
static void alert_entry(void *o) {
    printf("[STATE] Entered ALERT! Sending emergency payload to server!\n");
    fsm.previous_state = STATE_ALERT;
    
    // Turn on the Red LED and Buzzer for immediate local feedback
    sensors_led_on(LED_RED);
    //sensors_buzzer_on();
    
    // Call comms.c to initiate the panic localization and publish
    //comms_send_alert(REASON_BUTTON_PRESSED);
    comms_safe_disconnect(); 
}

static enum smf_state_result alert_run(void *o) {

    if (fsm.current_event.type == EVENT_SERVER_ACK_ALERT) {
        // Once the server acknowledges, turn off the hardware and sleep
        sensors_led_off(LED_RED);
        sensors_buzzer_off();
        
        // Clear the panic/fall status so next pings say "ok"
        comms_clear_alert(); 
        
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    }
    return SMF_EVENT_HANDLED;
}



/* ======================================================================
 * STATE TABLE MAPPING
 * ====================================================================== */


// smf_create_state parameters: (entry, run, exit, parent, custom_data)
static const struct smf_state states[] = {
    [STATE_DEEP_SLEEP]      = SMF_CREATE_STATE(deep_sleep_entry, deep_sleep_run, deep_sleep_exit, NULL, NULL),
    [STATE_LOCATION_PING]   = SMF_CREATE_STATE(location_ping_entry, location_ping_run, NULL, NULL, NULL),
    [STATE_ACTIVE_TRACKING] = SMF_CREATE_STATE(active_tracking_entry, active_tracking_run, active_tracking_exit, NULL, NULL),
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
        if (zbus_sub_wait_msg(&fsm_sub, &chan, &fsm.current_event, K_FOREVER) == 0) {
            
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
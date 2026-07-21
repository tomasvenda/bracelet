/*
* PART OF MASTER'S THESIS: Design of an End-to-End IoT System for Monitoring Vulnerable Users 
 
 
 * fsm.c -- Central bracelet state machine (Zephyr SMF) driven over ZBUS.
 * Owns DEEP_SLEEP / LOCATION_PING / ACTIVE_TRACKING / EVALUATION / ALERT
 * states, all associated timers (tracking, cooldown, per-state watchdog,
 * alert beep, alert cancel window), and the button/IMU event routing that
 * decides which state transitions to trigger. 
 */

#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>
#include <stdio.h>
#include "events.h"
#include "sensors.h"
#include "comms.h"
#include "ml_wrapper.h"

/* Defining the ZBUS channel */
ZBUS_CHAN_DEFINE(fsm_events_chan,           /* Name */
                 struct bracelet_event,     /* Payload type */
                 NULL,                      /* Validator */
                 NULL,                      /* User data */
                 ZBUS_OBSERVERS(fsm_sub),   /* Initial observers */
                 ZBUS_MSG_INIT(0));         /* Initial value */

/* Define the ZBUS subscriber for the FSM thread */
ZBUS_MSG_SUBSCRIBER_DEFINE(fsm_sub);

/* 1 minute timer for active tracking */
static void tracking_timer_expiry_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(tracking_timer, tracking_timer_expiry_cb, NULL);

static void tracking_timer_expiry_cb(struct k_timer *timer_id) {
    struct bracelet_event event = { .type = EVENT_TIMER_1MIN_EXPIRED };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

/* 10 minute timer for deep sleep */
static void cooldown_timer_expiry_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(cooldown_timer, cooldown_timer_expiry_cb, NULL);

static void cooldown_timer_expiry_cb(struct k_timer *timer_id) {
    struct bracelet_event event = { .type = EVENT_TIMER_10MIN_EXPIRED };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

/* Short timer used for the window in which the user can cancel an alert */
static void alert_confirm_timer_cb(struct k_timer *timer_id) {
    struct bracelet_event event = { .type = EVENT_ALERT_CANCEL_WINDOW_TIMEOUT };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}
K_TIMER_DEFINE(alert_confirm_timer, alert_confirm_timer_cb, NULL);

/*===========================*/
/* Per-state watchdog timer  */
/*===========================*/
static void state_timeout_cb(struct k_timer *timer_id) {
    struct bracelet_event event = { .type = EVENT_STATE_TIMEOUT };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}
K_TIMER_DEFINE(state_timeout_timer, state_timeout_cb, NULL);

/*===============================================================*/
/* ALERT beep pattern: 200 ms ON / 1800 ms OFF.                  */
/*===============================================================*/
static bool beep_is_on;
static void beep_timer_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(alert_beep_timer, beep_timer_cb, NULL);

static void beep_work_fn(struct k_work *work) {
    if (beep_is_on) {
        sensors_buzzer_off();
        beep_is_on = false;
        k_timer_start(&alert_beep_timer, K_MSEC(1800), K_NO_WAIT);
    } else {
        sensors_buzzer_on();
        beep_is_on = true;
        k_timer_start(&alert_beep_timer, K_MSEC(200), K_NO_WAIT);
    }
}
K_WORK_DEFINE(alert_beep_work, beep_work_fn);

static void beep_timer_cb(struct k_timer *timer_id) {
    k_work_submit(&alert_beep_work);
}

static void alert_beep_start(void) {
    beep_is_on = false;
    k_work_submit(&alert_beep_work);   /* first beep immediately */
}

static void alert_beep_stop(void) {
    k_timer_stop(&alert_beep_timer);
    sensors_buzzer_off();
    beep_is_on = false;
}

/* Remaining cooldown time */
/* Captured on exit from deep sleep so it can be resumed after evaluation finishes */
static uint32_t cooldown_remaining_ms;

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

/* The FSM Context */
struct fsm_context {
    struct smf_ctx smf;
    enum state_names previous_state;
    struct bracelet_event current_event;
} fsm;

/* Reason for entering ALERT: set by the button override or by a
 * confirmed ML fall, consumed by alert_entry(). */
static enum alert_reason pending_alert_reason = REASON_BUTTON_PRESSED;

/* ======================================================================
 * STATE ACTIONS
 * ====================================================================== */

// --- DEEP SLEEP ---
static void deep_sleep_entry(void *o) {
    printf("[STATE] Entered DEEP_SLEEP. MCU sleeping. LTE/GNSS off.\n");

    /* Ensure Blue LED turns off when going back to sleep */
    sensors_led_off(LED_BLUE);

    /* 1. Sever the connection gracefully to save maximum battery */
    comms_safe_disconnect();

    /* 2. Arm or disarm the motion trigger based on history */
    static bool is_first_run = true;
    if (is_first_run) {
        printf("[DEEP SLEEP] First boot detected. Enabling IMU wake immediately.\n");
        sensors_enable_motion_trigger();
        is_first_run = false;
    } 
    else if (fsm.previous_state == STATE_EVALUATION) {
        /* Restore exactly what was true before the impact interrupted the state */
        if (cooldown_remaining_ms > 0) {
            printf("[DEEP SLEEP] False alarm. Resuming cooldown (%u s left).\n",
                   cooldown_remaining_ms / 1000U);
            sensors_disable_motion_trigger();
            k_timer_start(&cooldown_timer, K_MSEC(cooldown_remaining_ms), K_NO_WAIT);
        } else {
            printf("[DEEP SLEEP] False alarm. Cooldown already over; re-arming IMU wake.\n");
            sensors_enable_motion_trigger();
        }
    } 
    else {
        printf("[DEEP SLEEP] Network cycle complete. Starting 10-minute cooldown timer.\n");
        sensors_disable_motion_trigger();
        k_timer_start(&cooldown_timer, K_MINUTES(10), K_NO_WAIT);
    }

    fsm.previous_state = STATE_DEEP_SLEEP;
}

static void deep_sleep_exit(void *o) {
    cooldown_remaining_ms = k_timer_remaining_get(&cooldown_timer);
    k_timer_stop(&cooldown_timer);
}

static enum smf_state_result deep_sleep_run(void *o) {
    if (fsm.current_event.type == EVENT_TIMER_10MIN_EXPIRED) {
        printf("[DEEP SLEEP] 10 min cooldown passed. Enabling IMU wake interrupt.\n");
        sensors_enable_motion_trigger();
    }
    else if (fsm.current_event.type == EVENT_IMU_LIGHT_MOTION) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_LOCATION_PING]);
    }
    return SMF_EVENT_HANDLED;
}

// --- LOCATION PING ---
#define LOC_PING_MAX_RETRIES 4
static uint8_t loc_ping_retries;
static bool    loc_ping_started;

static void location_ping_try_start(void) {
    if (comms_update_localization() == 0) {
        loc_ping_started = true;
        /* Never wait forever for the server verdict */
        k_timer_start(&state_timeout_timer, K_SECONDS(270), K_NO_WAIT);
    } else {
        loc_ping_started = false;
        printf("[LOCATION PING] Localization busy; retrying in 15 s (%u/%u).\n",
               loc_ping_retries + 1U, LOC_PING_MAX_RETRIES);
        k_timer_start(&state_timeout_timer, K_SECONDS(15), K_NO_WAIT);
    }
}

static void location_ping_entry(void *o) {
    printf("[STATE] Entered LOCATION_PING. Updating location...\n");
    fsm.previous_state = STATE_LOCATION_PING;

    sensors_led_on(LED_BLUE);

    sensors_disable_motion_trigger();

    loc_ping_retries = 0;
    location_ping_try_start();
}

static void location_ping_exit(void *o) {
    k_timer_stop(&state_timeout_timer);
}

static enum smf_state_result location_ping_run(void *o) {
    printf("[STATE] location_ping_run\n");

    if (fsm.current_event.type == EVENT_SERVER_REPLY_HOME) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    } else if (fsm.current_event.type == EVENT_SERVER_REPLY_AWAY) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
    } else if (fsm.current_event.type == EVENT_LOC_FAILURE) {
        printf("[LOCATION PING] No server verdict in time. Failing safe -> ACTIVE_TRACKING.\n");
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
    } else if (fsm.current_event.type == EVENT_STATE_TIMEOUT) {
        if (!loc_ping_started && loc_ping_retries < LOC_PING_MAX_RETRIES) {
            loc_ping_retries++;
            location_ping_try_start();
        } else {
            printf("[LOCATION PING] No server verdict in time. "
                   "Failing safe -> ACTIVE_TRACKING.\n");
            smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
        }
    }

    return SMF_EVENT_HANDLED;
}

// --- ACTIVE TRACKING ---
static void active_tracking_entry(void *o) {
    printf("[STATE] Entered ACTIVE_TRACKING. High alert tracking.\n");
    fsm.previous_state = STATE_ACTIVE_TRACKING;
    sensors_led_off(LED_BLUE);
    k_timer_start(&tracking_timer, K_MINUTES(3), K_NO_WAIT);   /* one-shot */
}

static void active_tracking_exit(void *o) {
    printf("[STATE] Exiting ACTIVE_TRACKING. Stopping timer.\n");
    k_timer_stop(&tracking_timer);
}

static enum smf_state_result active_tracking_run(void *o) {
    switch (fsm.current_event.type) {
    case EVENT_TIMER_1MIN_EXPIRED:
        printf("[ACTIVE TRACKING] Timer expired. Updating location...\n");
        if (comms_update_localization() != 0) {
            printf("[ACTIVE TRACKING] Localization busy; retry in 15 s.\n");
            k_timer_start(&tracking_timer, K_SECONDS(15), K_NO_WAIT);
        }
        break;

    case EVENT_LOC_SUCCESS:
    case EVENT_LOC_FAILURE:
        k_timer_start(&tracking_timer, K_MINUTES(3), K_NO_WAIT);
        break;
    
    case EVENT_SERVER_REPLY_HOME:
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
        break;

    default:
        break;
    }
    return SMF_EVENT_HANDLED;
}


/* ======================================================================
 * EVALUATION: capture the 3s fall window and run the TinyML model.
 *
 * The feature buffer is STATIC (2.4 KB) -- deliberately not on the
 * system workqueue stack. The capture itself blocks ~3.5 s inside the
 * workqueue; the FSM thread stays free to receive the panic button
 * (global override) the whole time.
 * ====================================================================== */
static float ml_features[600];   /* 150 frames x 4 axes */

static void do_ml_evaluation_work(struct k_work *work) {
    printf("[EVALUATION] Harsh impact detected! Capturing 3s event window...\n");

    /* 1. IMPACT TRIGGERED: Turn on Blue LED to indicate processing */
    sensors_led_on(LED_BLUE);

    struct bracelet_event event;

    int ret = sensors_capture_fall_window(ml_features, ARRAY_SIZE(ml_features));
    if (ret != 0) {
        printf("[EVALUATION] Window capture failed (%d). Treating as no-fall.\n", ret);
        
        /* NOT A FALL (Error): Turn off Blue, flash Green for 2 seconds */
        sensors_led_off(LED_BLUE);
        sensors_led_on(LED_GREEN);
        k_sleep(K_MSEC(2000));
        sensors_led_off(LED_GREEN);

        event.type = EVENT_ML_NO_FALL;
        zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
        sensors_evaluation_done();
        return;
    }

    int result = run_fall_inference(ml_features, ARRAY_SIZE(ml_features));

    /* Inference complete: Turn off Blue */
    sensors_led_off(LED_BLUE);

    if (result == 1) {
        /* FALL CONFIRMED: The FSM's alert_entry will immediately turn on RED */
        event.type = EVENT_ML_FALL_DETECTED;
    } else {
        if (result < 0) {
            printf("[EVALUATION] Inference error (%d). Treating as no-fall.\n", result);
        }
        /* NOT A FALL: Flash Green for 2 seconds */
        sensors_led_on(LED_GREEN);
        k_sleep(K_MSEC(2000));
        sensors_led_off(LED_GREEN);

        event.type = EVENT_ML_NO_FALL;
    }
    sensors_evaluation_done();
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

K_WORK_DEFINE(ml_eval_work, do_ml_evaluation_work);

static void evaluation_entry(void *o) {
    /* Toss the heavy data-collection and ML math to the background
     * worker so the FSM is free to instantly hear the panic button. */
    k_work_submit(&ml_eval_work);
}

static enum smf_state_result evaluation_run(void *o) {
    if (fsm.current_event.type == EVENT_ML_NO_FALL) {
        printf("        -> TinyML says False Alarm. Returning to previous state.\n");
        enum state_names origin = fsm.previous_state;
        fsm.previous_state = STATE_EVALUATION;   /* so deep_sleep_entry knows */
        smf_set_state(SMF_CTX(&fsm), &states[origin]);
    } else if (fsm.current_event.type == EVENT_ML_FALL_DETECTED) {
        printf("        -> TinyML CONFIRMED FALL. Raising alert!\n");
        pending_alert_reason = REASON_FALL_DETECTED;
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ALERT]);
    }

    return SMF_EVENT_HANDLED;
}


// --- ALERT (Emergency) ---
/* Leaving ALERT requires BOTH: server acked the panic message AND the
 * alert localization stack ran to completion. Order can be either. */
static bool alert_acked;
static bool alert_pending_confirm;

static void alert_entry(void *o) {
    printf("[STATE] Entered ALERT! 10s cancel window open...\n");
    alert_acked = false;
    alert_pending_confirm = true;

    /* Hardware indications */
    sensors_led_off(LED_BLUE);
    sensors_led_on(LED_RED);
    alert_beep_start(); 

    /* 10 seconds to cancel the alert */
    k_timer_start(&alert_confirm_timer, K_SECONDS(10), K_NO_WAIT);
}

static void alert_exit(void *o) {
    k_timer_stop(&state_timeout_timer);
    alert_beep_stop();
    sensors_led_off(LED_RED);
}

static enum smf_state_result alert_run(void *o) {
    if (alert_pending_confirm) {
        if (fsm.current_event.type == EVENT_BUTTON_LONG_PRESS) {
            printf("[ALERT] Cancelled by user long-press.\n");
            k_timer_stop(&alert_confirm_timer);
            alert_beep_stop();
            sensors_led_off(LED_RED);
            enum state_names origin = fsm.previous_state;
            smf_set_state(SMF_CTX(&fsm), &states[origin]);
            return SMF_EVENT_HANDLED;
        }
        if (fsm.current_event.type == EVENT_ALERT_CANCEL_WINDOW_TIMEOUT) {
            printf("[ALERT] No cancel received. Proceeding with alert pipeline.\n");
            alert_pending_confirm = false;
            comms_send_alert_status(pending_alert_reason);
            k_timer_start(&state_timeout_timer, K_SECONDS(60), K_SECONDS(60));
        }
        return SMF_EVENT_HANDLED; /* swallow everything else while pending */
    }
    
    if (fsm.current_event.type == EVENT_SERVER_ACK_ALERT) {
        if (!alert_acked) {
            printf("[ALERT] Server ACK received! Stopping buzzer & starting localization...\n");
            alert_acked = true;
            alert_beep_stop();
            comms_clear_alert();

            if (comms_update_localization() != 0) {
                printf("[ALERT] Localization busy; retrying in 15 s.\n");
                k_timer_start(&state_timeout_timer, K_SECONDS(15), K_NO_WAIT);
            } else {
                /* Watchdog -- the loc thread must answer inside this. */
                k_timer_start(&state_timeout_timer, K_MINUTES(5), K_NO_WAIT);
            }
        }
    }
    
    else if (fsm.current_event.type == EVENT_LOC_SUCCESS) {
        printf("[ALERT] Localization successful. Exiting alert state.\n");
        
        /* Return to whatever we were doing before the emergency */
        if (fsm.previous_state == STATE_DEEP_SLEEP) {
            smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
        } else {
            smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
        }
    } 
    
    else if (fsm.current_event.type == EVENT_LOC_FAILURE) {
        printf("[ALERT] Localization failed (timeout/no verdict). Forcing ACTIVE_TRACKING.\n");
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
    } 
    
    else if (fsm.current_event.type == EVENT_STATE_TIMEOUT) {
        if (!alert_acked) {
            printf("[ALERT] No server ACK yet. Re-sending alert status.\n");
            comms_send_alert_status(pending_alert_reason);
        } else {
            printf("[ALERT] Retrying localization.\n");
            if (comms_update_localization() != 0) {
                k_timer_start(&state_timeout_timer, K_SECONDS(15), K_NO_WAIT);
            } else {
                k_timer_start(&state_timeout_timer, K_MINUTES(5), K_NO_WAIT);
            }
        }
    }

    return SMF_EVENT_HANDLED;
}

/* ======================================================================
 * STATE TABLE MAPPING
 * ====================================================================== */
static const struct smf_state states[] = {
    [STATE_DEEP_SLEEP]      = SMF_CREATE_STATE(deep_sleep_entry, deep_sleep_run, deep_sleep_exit, NULL, NULL),
    [STATE_LOCATION_PING]   = SMF_CREATE_STATE(location_ping_entry, location_ping_run, location_ping_exit, NULL, NULL),
    [STATE_ACTIVE_TRACKING] = SMF_CREATE_STATE(active_tracking_entry, active_tracking_run, active_tracking_exit, NULL, NULL),
    [STATE_EVALUATION]      = SMF_CREATE_STATE(evaluation_entry, evaluation_run, NULL, NULL, NULL),
    [STATE_ALERT]           = SMF_CREATE_STATE(alert_entry, alert_run, alert_exit, NULL, NULL),
};

/* ======================================================================
 * MAIN FSM THREAD (Listens to ZBUS)
 * ====================================================================== */
void fsm_thread_main(void)
{
    const struct zbus_channel *chan;

    smf_set_initial(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);

    while (1) {
        if (zbus_sub_wait_msg(&fsm_sub, &chan, &fsm.current_event, K_FOREVER) == 0) {

            if (fsm.current_event.type == EVENT_BUTTON_PRESSED) {
                if (SMF_CTX(&fsm)->current == &states[STATE_ALERT]) {
                    printf("[FSM] Button ignored: already in ALERT.\n");
                    continue;
                }

                pending_alert_reason = REASON_BUTTON_PRESSED;
                
                smf_set_state(SMF_CTX(&fsm), &states[STATE_ALERT]);
                continue;
            }
            else if (fsm.current_event.type == EVENT_IMU_HARSH_IMPACT) {
                smf_set_state(SMF_CTX(&fsm), &states[STATE_EVALUATION]);
                continue;
            }

            smf_run_state(SMF_CTX(&fsm));
        }
    }
}

K_THREAD_DEFINE(fsm_tid, 2048, fsm_thread_main, NULL, NULL, NULL, 5, 0, K_TICKS_FOREVER);
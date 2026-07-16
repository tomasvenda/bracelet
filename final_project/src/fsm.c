#include <zephyr/kernel.h>
#include <zephyr/smf.h>
#include <zephyr/zbus/zbus.h>
#include <stdio.h>
#include "events.h"
#include "sensors.h"
#include "comms.h"
#include "ml_wrapper.h"


/* Define the ZBUS channel (Message size, subscribers, etc.) */
ZBUS_CHAN_DEFINE(fsm_events_chan,       /* Name */
                 struct bracelet_event, /* Payload type */
                 NULL,                  /* Validator */
                 NULL,                  /* User data */
                 ZBUS_OBSERVERS(fsm_sub), /* Initial observers */
                 ZBUS_MSG_INIT(0));     /* Initial value */

/* Define the ZBUS subscriber for the FSM thread */
ZBUS_MSG_SUBSCRIBER_DEFINE(fsm_sub);

/*===========================*/
/* Timer for active tracking */
/*===========================*/
static void tracking_timer_expiry_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(tracking_timer, tracking_timer_expiry_cb, NULL);

static void tracking_timer_expiry_cb(struct k_timer *timer_id) {
    struct bracelet_event event = { .type = EVENT_TIMER_1MIN_EXPIRED };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

/*===========================*/
/* Timer for deep sleep      */
/*===========================*/
static void cooldown_timer_expiry_cb(struct k_timer *timer_id);
K_TIMER_DEFINE(cooldown_timer, cooldown_timer_expiry_cb, NULL);

static void cooldown_timer_expiry_cb(struct k_timer *timer_id) {
    struct bracelet_event event = { .type = EVENT_TIMER_10MIN_EXPIRED };
    zbus_chan_pub(&fsm_events_chan, &event, K_NO_WAIT);
}

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
/* A continuous buzzer + red LED + nRF7002 scan TX bursts        */
/* browned out the board; pulsing removes the standing load and  */
/* makes overlap with radio TX peaks unlikely.                   */
/* Timer fires in ISR context, so the buzzer driver is driven    */
/* from a work item instead.                                     */
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

/* Remaining cooldown time captured on exit, so a false-alarm
 * round-trip through EVALUATION can resume it instead of losing it. */
static uint32_t cooldown_remaining_ms;

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

/* Why we are entering ALERT: set by the button override or by a
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
        /* Restore exactly what was true before the impact interrupted us:
         * if the 10-min cooldown was still pending, resume it with the
         * remaining time (trigger stays disarmed); if it had already
         * expired, the motion trigger was armed, so re-arm it. */
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
static void location_ping_entry(void *o) {
    printf("[STATE] Entered LOCATION_PING. Updating location...\n");
    fsm.previous_state = STATE_LOCATION_PING;

    /* LIGHT MOTION TRIGGERED: Turn on Blue LED while the network works */
    sensors_led_on(LED_BLUE);

    sensors_disable_motion_trigger();
    comms_update_localization();
    /* Never wait forever for the server verdict */
    k_timer_start(&state_timeout_timer, K_SECONDS(270), K_NO_WAIT);
}

static void location_ping_exit(void *o) {
    k_timer_stop(&state_timeout_timer);
}

static enum smf_state_result location_ping_run(void *o) {
    printf("[STATE] location_ping_run");

    if (fsm.current_event.type == EVENT_SERVER_REPLY_HOME) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_DEEP_SLEEP]);
    } else if (fsm.current_event.type == EVENT_SERVER_REPLY_AWAY) {
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
    } else if (fsm.current_event.type == EVENT_STATE_TIMEOUT) {
        printf("[LOCATION PING] No server verdict in time. Failing safe -> ACTIVE_TRACKING.\n");
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
    }

    return SMF_EVENT_HANDLED;
}

// --- ACTIVE TRACKING ---
static void active_tracking_entry(void *o) {
    printf("[STATE] Entered ACTIVE_TRACKING. High alert tracking.\n");
    fsm.previous_state = STATE_ACTIVE_TRACKING;
    /* HYGIENE: Ensure Blue LED turns off when entering tracking */
    sensors_led_off(LED_BLUE);
    k_timer_start(&tracking_timer, K_MINUTES(1), K_MINUTES(1));
}

static void active_tracking_exit(void *o) {
    printf("[STATE] Exiting ACTIVE_TRACKING. Stopping timer.\n");
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
static bool alert_stack_done;

static void alert_entry(void *o) {
    printf("[STATE] Entered ALERT! Sending emergency payload to server!\n");
    fsm.previous_state = STATE_ALERT;
    alert_acked = false;
    alert_stack_done = false;

    /* Turn OFF Blue (in case of panic button) and turn ON Red */
    sensors_led_off(LED_BLUE);
    sensors_led_on(LED_RED);
    alert_beep_start();   /* pulsed: 200 ms on / 1.8 s off */

    comms_send_alert(pending_alert_reason);
    /* Re-send the alert every 60 s until the server acks */
    k_timer_start(&state_timeout_timer, K_SECONDS(60), K_SECONDS(60));
}

static void alert_exit(void *o) {
    k_timer_stop(&state_timeout_timer);
    alert_beep_stop();   /* never let the beeper outlive ALERT */
}

static enum smf_state_result alert_run(void *o) {

    if (fsm.current_event.type == EVENT_SERVER_ACK_ALERT) {
        /* ACK = "panic received", NOT "emergency over". Silence the
         * alarm and stop the 60 s re-sends, but the localization
         * stack keeps running to completion. */
        printf("[ALERT] Server acked the panic. Alarm off; waiting for stack.\n");
        alert_acked = true;
        sensors_led_off(LED_RED);
        alert_beep_stop();
        k_timer_stop(&state_timeout_timer);
    } else if (fsm.current_event.type == EVENT_LOC_DONE) {
        printf("[ALERT] Alert localization stack completed.\n");
        alert_stack_done = true;
    } else if (fsm.current_event.type == EVENT_STATE_TIMEOUT) {
        printf("[ALERT] No server ack yet. Re-sending alert.\n");
        comms_send_alert(pending_alert_reason);
    }

    if (alert_acked && alert_stack_done) {
        /* Panic delivered + stack finished (with or without a
         * location). Emergency mode continues as high-rate tracking. */
        comms_clear_alert();
        smf_set_state(SMF_CTX(&fsm), &states[STATE_ACTIVE_TRACKING]);
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

            /* NO zbus_chan_read() here -- and delete it if present.
             * wait_msg has ALREADY delivered the queued message into
             * fsm.current_event. A chan_read would overwrite it with
             * the channel's LATEST value, so two rapid events (e.g.
             * BUTTON right after LIGHT_MOTION) would process the
             * newest twice and silently drop the older one. */

            if (fsm.current_event.type == EVENT_BUTTON_PRESSED) {
                /* Already in ALERT: ignore repeat presses. Re-entering
                 * would reset the acked/stack-done flags, restart the
                 * beeper, and queue a redundant waterfall. One alert
                 * at a time; it ends via ack + stack completion. */
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
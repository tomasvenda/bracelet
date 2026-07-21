/* sensors.h -- sensor subsystem public API.
 *
 * The BMI270 is now driven through the Bosch
 * legacy API (raw I2C), NOT the Zephyr sensor driver. This is required
 * because the model needs the low-g feature (harsh-impact trigger) and
 * the hardware FIFO (the 1.5 s of PRE-impact data the model was
 * trained on) -- neither exists in the Zephyr bosch,bmi270 driver.
 * sensors_collect_ml_window() is replaced by
 * sensors_capture_fall_window(), which reproduces the training-data
 * pipeline exactly.
 */

#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stddef.h>

/* LED color bitmasks (PWM-driven) */
#define LED_RED   BIT(0)
#define LED_GREEN BIT(1)
#define LED_BLUE  BIT(2)

#define LED_YELLOW (LED_RED | LED_GREEN)
#define LED_CYAN   (LED_GREEN | LED_BLUE)

int  sensors_init(void);

/* Light-motion (any-motion) trigger control -- used by the FSM wake 
 * up during DEEP_SLEEP. The low-g harsh-impact trigger is
 * NOT affected by these: fall detection stays armed at all times. */
void sensors_enable_motion_trigger(void);
void sensors_disable_motion_trigger(void);

/* Capture one 3-second fall window (1.5 s pre-impact from the FIFO +
 * 1.5 s post-impact) and write it as interleaved model features:
 *   [x0,y0,z0,dp0, x1,y1,z1,dp1, ...]
 * accel in g, dp = pressure delta in hPa from the first sample.
 * 'count' must be 600 (150 frames x 4 axes).
 * Blocks for ~3.5 s. Returns 0 on success, negative on short capture.
 * Rearms the low-g trigger before returning. */
int  sensors_capture_fall_window(float *features, size_t count);

/* LED & buzzer */
/* Turns on one or more LED colors (LED_RED/GREEN/BLUE). */
void sensors_led_on(uint8_t color);

/* Turns off one or more LED colors. */
void sensors_led_off(uint8_t color);

/* Turns the buzzer PWM output off or on at fixed duty cycle. */
void sensors_buzzer_on(void);
void sensors_buzzer_off(void);

/* Clears the capture-pending flag, re-enabling harsh-impact interrupt
 * handling. Must be called after a fall-window capture + ML inference
 * cycle completes, regardless of outcome. */
void sensors_evaluation_done(void);

/* Reads battery voltage from the nPM1300 charger/gauge and converts it
 * to an approximate percentage by preset values in a voltage lookup table. */
int get_battery_level(void);

/* Startup indicator: blinks BLUE/RED alternately until setup completes.
 * Call sensors_startup_blink_start() before init work begins, and
 * sensors_startup_blink_stop() once sensors_init()/comms_init() finish. */
void sensors_startup_blink_start(void);
void sensors_startup_blink_stop(void);

/* Loclaization indicator: blinks yellow for localization in progress.
 * Call sensors_localization_blink_start() when a localization attempt
 * begins, and sensors_localization_blink_stop() when it ends
 * (success, failure, or timeout). */
void sensors_localization_blink_start(void);
void sensors_localization_blink_stop(void);

/* One-shot notification blinks. Both block briefly (~600ms) and should
 * only be called from contexts where that's acceptable (state entry
 * functions, not ISRs). */
void sensors_notify_deep_sleep(void);      /* 2x green blinks */
void sensors_notify_active_tracking(void); /* 2x cyan blinks  */

#endif /* SENSORS_H */
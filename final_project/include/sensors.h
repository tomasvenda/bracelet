/* sensors.h -- sensor subsystem public API.
 *
 * CHANGED vs. old version: the BMI270 is now driven through the Bosch
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

int  sensors_init(void);

/* Light-motion (any-motion) trigger control -- used by the FSM to
 * gate wake-ups during cooldown. The low-g harsh-impact trigger is
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

/* LED & buzzer (PWM, unchanged) */
void sensors_led_on(uint8_t color);
void sensors_led_off(uint8_t color);
void sensors_buzzer_on(void);
void sensors_buzzer_off(void);
void sensors_evaluation_done(void);

#endif /* SENSORS_H */
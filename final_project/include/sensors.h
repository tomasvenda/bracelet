#ifndef SENSORS_H
#define SENSORS_H

/* Initializes the IMU, Barometer, and Button, and arms the hardware interrupts */
int sensors_init(void);

/* * Called by the background worker during STATE_EVALUATION.
 * Blocks and collects a time-series window of data for the TinyML model.
 * * Assumes 50Hz sampling for 2 seconds = 100 samples.
 * Each sample has 8 features: Accel(X,Y,Z), Gyro(X,Y,Z), Pressure, Temperature
 * Total buffer size expected: 800 floats.
 */
int sensors_collect_ml_window(float *ml_data_buffer, int max_features);

// Color bitmask — include this header wherever you call sensors_led_on/off
#define LED_RED   BIT(0)
#define LED_GREEN BIT(1)
#define LED_BLUE  BIT(2)

void sensors_led_on(uint8_t color);
void sensors_led_off(uint8_t color);
void sensors_buzzer_on(void);
void sensors_buzzer_off(void);

void sensors_disable_motion_trigger(void);
void sensors_enable_motion_trigger(void);

#endif /* SENSORS_H */
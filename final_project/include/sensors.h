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

#endif /* SENSORS_H */
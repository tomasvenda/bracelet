#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "app_events.h"

LOG_MODULE_REGISTER(low_power_acl, LOG_LEVEL_INF);

// Define the global event object here 
K_EVENT_DEFINE(app_events);

// Timeout
#define INACTIVITY_TIMEOUT_S 30

// Forward declaration of timer callback 
static void inactivity_timer_handler(struct k_timer *timer_id);

// Define the RTOS timer
K_TIMER_DEFINE(inactivity_timer, inactivity_timer_handler, NULL);

// THE GO-TO-SLEEP HANDLER (Runs when the timer expires)

static void inactivity_timer_handler(struct k_timer *timer_id)
{
    LOG_INF("No movement for %d seconds. System going to sleep...", INACTIVITY_TIMEOUT_S);
    
    // Clear the flag. This tells the WiFi thread to pause!
    k_event_clear(&app_events, EVENT_USER_MOVING);
}

// THE HARDWARE INTERRUPT HANDLER (Runs the microsecond the sensor moves)
 
static void adxl367_trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
    /* We only care about motion triggers */
    if (trigger->type == SENSOR_TRIG_MOTION || trigger->type == SENSOR_TRIG_DELTA) {
        
        /* If we weren't already moving, announce it and set the flag! */
        if ((k_event_test(&app_events, EVENT_USER_MOVING) & EVENT_USER_MOVING) == 0) {
            LOG_INF("Hardware Interrupt: Movement detected! Waking system...");
            k_event_post(&app_events, EVENT_USER_MOVING);
        }

        /* Reset the inactivity timer back to 30 seconds */
        k_timer_start(&inactivity_timer, K_SECONDS(INACTIVITY_TIMEOUT_S), K_NO_WAIT);
    }
}

// INITIALIZATION function to set up the ADXL367 and its interrupt at boot
 
int low_power_acl_init(void)
{
    const struct device *const dev = DEVICE_DT_GET_ONE(adi_adxl367);
    struct sensor_value full_scale, threshold;

    if (!device_is_ready(dev)) {
        LOG_ERR("ADXL367 is not ready! Check Devicetree overlay.");
        return -ENODEV;
    }

    /* Set scale to 2G */
    full_scale.val1 = 2;            
    full_scale.val2 = 0;
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    
    /* Configure the hardware motion threshold (e.g., 1 m/s^2) 
       This tells the physical ADXL chip how hard it needs to be bumped to fire the interrupt */
    threshold.val1 = 1; 
    threshold.val2 = 0;
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_UPPER_THRESH, &threshold);

    /* Set up the Trigger */
    struct sensor_trigger trig = {
        .type = SENSOR_TRIG_MOTION, /* Or SENSOR_TRIG_DELTA, depending on Zephyr driver version */
        .chan = SENSOR_CHAN_ACCEL_XYZ,
    };

    /* Attach our callback function to the hardware interrupt */
    if (sensor_trigger_set(dev, &trig, adxl367_trigger_handler) < 0) {
        LOG_ERR("Could not set ADXL367 trigger! Did you configure int1-gpios in the overlay?");
        return -EIO;
    }

    LOG_INF("ADXL367 Hardware Interrupts configured successfully.");
    return 0;
}

/* Zephyr macro to run this initialization automatically at boot */
SYS_INIT(low_power_acl_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
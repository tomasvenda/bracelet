#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "app_events.h"

LOG_MODULE_REGISTER(low_power_acl, LOG_LEVEL_INF);

/* The global event object that the WiFi thread listens to */
K_EVENT_DEFINE(app_events);

#define INACTIVITY_TIMEOUT_S 30

static void inactivity_timer_handler(struct k_timer *timer_id);
K_TIMER_DEFINE(inactivity_timer, inactivity_timer_handler, NULL);

/* CRITICAL FIX: The trigger struct MUST be static so it survives in memory forever! */
static struct sensor_trigger trig;

/* 1. Timer expires -> User is asleep -> Clear flag */
static void inactivity_timer_handler(struct k_timer *timer_id)
{
    LOG_INF("No movement for %d seconds. System going to sleep...", INACTIVITY_TIMEOUT_S);
    k_event_clear(&app_events, EVENT_USER_MOVING);
}

/* 2. Hardware interrupt fires -> User is moving -> Set flag */
static void adxl367_trigger_handler(const struct device *dev, const struct sensor_trigger *trigger)
{
    /* Since we fixed the memory pointer, we don't even need to check the trigger type. 
     * If this function runs, we know the ADXL367 physical pin fired! 
     */
    uint32_t current_state = k_event_test(&app_events, EVENT_USER_MOVING);

    /* If the system was asleep, wake it up! */
    if ((current_state & EVENT_USER_MOVING) == 0) {
        LOG_INF("Movement detected! Waking system...");
        k_event_post(&app_events, EVENT_USER_MOVING);
    }

    /* Every time we move, reset the 30-second sleep timer */
    k_timer_start(&inactivity_timer, K_SECONDS(INACTIVITY_TIMEOUT_S), K_NO_WAIT);
}

/* 3. Setup function called by main.c */
int low_power_acl_init(void)
{
    const struct device *const dev = DEVICE_DT_GET_ONE(adi_adxl367);
    if (!device_is_ready(dev)) {
        LOG_ERR("ADXL367 is not ready! Check Devicetree.");
        return -ENODEV;
    }

    /* Configure our permanent static trigger */
    trig.type = SENSOR_TRIG_THRESHOLD;
    trig.chan = SENSOR_CHAN_ACCEL_XYZ;

    if (sensor_trigger_set(dev, &trig, adxl367_trigger_handler) < 0) {
        LOG_ERR("Could not set ADXL367 trigger!");
        return -EIO;
    }

    LOG_INF("ADXL367 Hardware Watchdog armed!");
    return 0;
}
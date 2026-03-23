#ifndef APP_EVENTS_H
#define APP_EVENTS_H

#include <zephyr/kernel.h>

/* The global event object. 
 * It is defined in low_power_acl.c, but we declare it 'extern' here 
 * so the WiFi thread can see it. 
 */
extern struct k_event app_events;

/* Event Flags */
#define EVENT_USER_MOVING BIT(0)

#endif /* APP_EVENTS_H */
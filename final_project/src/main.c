/*
 * BRACELET TRACKER - MAIN ENTRY POINT
 */

#include <zephyr/kernel.h>
#include <stdio.h>

/* Include our module APIs */
#include "sensors.h"
#include "comms.h"

int main(void)
{
    printf("========================================\n");
    printf("      BRACELET TRACKER BOOTING...       \n");
    printf("========================================\n");

    /* 1. Initialize Hardware Sensors & Interrupts */
    if (sensors_init() != 0) {
        printf("[FATAL] Sensor initialization failed! Check PMIC/I2C.\n");
        return -1;
    }
    
    //DEBUGGING
    //printf("DOING ONE WIFI SCAN HERE");
    //do_wifi_scan();

    /* 2. Initialize Modem, LTE, and MQTT */
    if (comms_init() != 0) {
        printf("[FATAL] Communications initialization failed! Check SIM/Antenna.\n");
        return -1;
    }
    
    printf("========================================\n");
    printf("        SYSTEM BOOT COMPLETE            \n");
    printf("   FSM is now listening for events.     \n");
    printf("========================================\n");

    /* * Because our FSM was created with K_THREAD_DEFINE, it is already 
     * running in the background. Our work queues for localization and 
     * ML evaluation are also waiting in the background.
     * * The main thread's job is done. We put it to sleep forever to save battery.
     */
    k_sleep(K_FOREVER);
    
    return 0;
}
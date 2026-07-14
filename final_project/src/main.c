/*
 * BRACELET TRACKER - MAIN ENTRY POINT
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/init.h>

/* Include our module APIs */
#include "sensors.h"
#include "comms.h"

extern const k_tid_t fsm_tid;

/* ======================================================================
 * MAIN ENTRY POINT
 * ====================================================================== */
int main(void)
{

    printf("========================================\n");
    printf("      BRACELET TRACKER BOOTING...       \n");
    printf("========================================\n");

    /* 1. Initialize Hardware Sensors & Interrupts */
    /* (Make sure you removed the ldo1_dev code from inside here!) */
    if (sensors_init() != 0) {
        printf("[FATAL] Sensor initialization failed! Check PMIC/I2C.\n");
        return -1;
    }
    printf("MAIN: sensors_init OK\n");

    /* 2. Initialize Modem, LTE, and MQTT */
    if (comms_init() != 0) {
        printf("[FATAL] Communications initialization failed! Check SIM/Antenna.\n");
        return -1;
    }
    printf("MAIN: comms_init OK\n");
    
    printf("========================================\n");
    printf("        SYSTEM BOOT COMPLETE            \n");
    printf("   FSM is now listening for events.     \n");
    printf("========================================\n");

    /* Start the FSM */
    k_thread_start(fsm_tid);

    /* Main thread goes to sleep to save power */
    k_sleep(K_FOREVER);
    
    return 0;
}
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

#if defined(CONFIG_APP_COLLECT_DATA)
    printf("MAIN: comms skipped (data-collection build)\n");
#else
    if (comms_init() != 0) {
        printf("[FATAL] Communications initialization failed! Check SIM/Antenna.\n");
        return -1;
    }
    printf("MAIN: comms_init OK\n");
#endif
    

    
    printf("========================================\n");
    printf("        SYSTEM BOOT COMPLETE            \n");
    printf("   FSM is now listening for events.     \n");
    printf("========================================\n");


#ifdef CONFIG_APP_POWER_TEST
    extern void power_test_run(void);
    power_test_run();
#elif defined(CONFIG_APP_DETECTOR_POWER_TEST)
    extern void detector_power_test_run(void);
    detector_power_test_run();
#elif defined(CONFIG_APP_DETECTOR_ACCURACY_TEST)
    extern void detector_accuracy_test_run(void);
    detector_accuracy_test_run();
#elif defined(CONFIG_APP_GNSS_POWER_TEST)
    gnss_power_test_run();
#elif defined(CONFIG_APP_DETECTOR_ACCURACY_TEST)
    extern void detector_accuracy_test_run(void);
    detector_accuracy_test_run();
#elif defined(CONFIG_APP_COLLECT_DATA)
    extern void collect_data_run(void);
    collect_data_run();
#else
    k_thread_start(fsm_tid);
    k_sleep(K_FOREVER);
#endif
    
    return 0;
}


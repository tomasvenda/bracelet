/* Temporary LTE power comparison test.
 * Mode A (PSM): modem stays registered, drops to PSM between cycles.
 * Mode B (OFF): modem CFUN=0 between cycles, full re-attach on wake.
 * Toggle with CONFIG_APP_POWER_TEST_PSM. */

#ifdef CONFIG_APP_POWER_TEST

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>
#include "comms.h"
#include "sensors.h"

LOG_MODULE_REGISTER(power_test, LOG_LEVEL_INF);

#define SLEEP_MINUTES   5
#define NUM_CYCLES      10

static void pt_lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
#if defined(CONFIG_LTE_LC_PSM_MODULE)
    case LTE_LC_EVT_PSM_UPDATE:
        LOG_INF("[PT] PSM grant: TAU=%d s, Active time=%d s",
                evt->psm_cfg.tau, evt->psm_cfg.active_time);
        if (evt->psm_cfg.active_time < 0) {
            LOG_WRN("[PT] NETWORK DID NOT GRANT PSM — PSM test invalid!");
        }
        break;
#endif
#if defined(CONFIG_LTE_LC_MODEM_SLEEP_MODULE)
    case LTE_LC_EVT_MODEM_SLEEP_ENTER:
        LOG_INF("[PT] Modem sleep ENTER (type %d) @ %lld ms",
                evt->modem_sleep.type, k_uptime_get());
        break;
    case LTE_LC_EVT_MODEM_SLEEP_EXIT:
        LOG_INF("[PT] Modem sleep EXIT @ %lld ms", k_uptime_get());
        break;
#endif
    case LTE_LC_EVT_RRC_UPDATE:
        LOG_INF("[PT] RRC %s @ %lld ms",
                evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "CONNECTED" : "IDLE",
                k_uptime_get());
        break;
    default:
        break;
    }
}

void power_test_run(void)
{
    lte_lc_register_handler(pt_lte_handler);

    /* Hold the rest of the board in the same config as your measured
     * deep sleep so the 1.10 mA floor is identical in both modes. */
    sensors_led_off(LED_BLUE);
    sensors_disable_motion_trigger();

    LOG_INF("[PT] Mode: %s | %d cycles of %d min sleep",
            IS_ENABLED(CONFIG_APP_POWER_TEST_PSM) ? "PSM" : "MODEM OFF",
            NUM_CYCLES, SLEEP_MINUTES);

    comms_mqtt_disconnect();
    if (!IS_ENABLED(CONFIG_APP_POWER_TEST_PSM)) {
        comms_safe_disconnect();          /* CFUN=0 */
    }
    k_sleep(K_MINUTES(SLEEP_MINUTES));

    for (int i = 1; i <= NUM_CYCLES; i++) {
        int64_t t0 = k_uptime_get();
        LOG_INF("[PT] ===== Cycle %d WAKE @ %lld ms =====", i, t0);

        /* In PSM mode the modem exits PSM automatically the moment the
         * socket is touched. In OFF mode ensure_connected() runs
         * lte_lc_normal() and blocks on registration. */
        int err = comms_mqtt_ensure_connected();
        if (err == 0) {
            comms_mqtt_publish("{\"status\":\"power_test\"}");
            k_sleep(K_SECONDS(3));        /* let the QoS1 PUBACK land */
        } else {
            LOG_ERR("[PT] Cycle %d: connect failed (%d)", i, err);
        }
        LOG_INF("[PT] Cycle %d wake phase: %lld ms", i, k_uptime_get() - t0);

        /* --- Back to sleep --- */
        comms_mqtt_disconnect();
        if (!IS_ENABLED(CONFIG_APP_POWER_TEST_PSM)) {
            comms_safe_disconnect();      /* CFUN=0, same as FSM deep sleep */
        }
        /* PSM mode: do nothing — modem enters PSM after the network's
         * RRC inactivity timer releases the connection (~5-20 s). */
        LOG_INF("[PT] ===== Cycle %d SLEEP @ %lld ms =====", i, k_uptime_get());
        k_sleep(K_MINUTES(SLEEP_MINUTES));
    }

    LOG_INF("[PT] Test complete. Halting.");
    if (!IS_ENABLED(CONFIG_APP_POWER_TEST_PSM)) {
        comms_safe_disconnect();
    }
    k_sleep(K_FOREVER);
}

#endif /* CONFIG_APP_POWER_TEST */
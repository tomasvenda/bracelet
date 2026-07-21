#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pmic_test, LOG_LEVEL_INF);

/* Change this to BUCK1 or BUCK2 if you are using a buck instead of LDSW1 */
const struct device *wifi_en_reg = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldsw1));

int main(void)
{
    int ret;
    int disable_attempts = 0;

    /* Give PMIC time to fully boot over I2C */
    k_msleep(1000); 

    LOG_INF("=== Starting nPM1300 Regulator Control Test ===");

    if (!device_is_ready(wifi_en_reg)) {
        LOG_ERR("Regulator device not ready! Check I2C lines and devicetree.");
        return -1;
    }

    /* =========================================
     * STRATEGY 1: DEFEAT REFERENCE COUNTING
     * ========================================= */
    LOG_INF("Phase 1: Draining Zephyr Regulator Reference Counter...");
    
    while (1) {
        ret = regulator_disable(wifi_en_reg);
        disable_attempts++;
        
        if (ret == 0) {
            LOG_WRN("Counter drained by 1. It was secretly enabled! (Attempt %d)", disable_attempts);
        } else {
            LOG_INF("Regulator fully disabled in software. Reached bottom of ref counter. (Attempts: %d)", disable_attempts);
            break;
        }

        /* Failsafe to prevent infinite loops if driver is broken */
        if (disable_attempts > 10) {
            LOG_ERR("Disable loop failed! Regulator driver refusing to shut down.");
            break;
        }
    }

    LOG_INF("--> PROBE NOW: Regulator should be OFF (0V). Waiting 30 seconds...");
    k_sleep(K_SECONDS(30));


    /* =========================================
     * STRATEGY 2: PERIODIC TOGGLE FOR PROBING
     * ========================================= */
    LOG_INF("Phase 2: Beginning 30-second toggle cycle.");
    
    while (1) {
        /* Turn ON */
        LOG_INF(">>> ENABLING Regulator (3.3V expected) <<<");
        ret = regulator_enable(wifi_en_reg);
        if (ret) {
            LOG_ERR("Failed to enable: %d", ret);
        }
        
        /* Wait 30 seconds for you to probe */
        k_sleep(K_SECONDS(30));

        /* Turn OFF */
        LOG_INF(">>> DISABLING Regulator (0V expected - discharging) <<<");
        ret = regulator_disable(wifi_en_reg);
        if (ret) {
            LOG_ERR("Failed to disable: %d", ret);
        }
        
        /* Wait 30 seconds for you to probe */
        k_sleep(K_SECONDS(30));
    }

    return 0;
}
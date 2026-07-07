#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <zephyr/drivers/regulator.h>

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

static K_SEM_DEFINE(lte_connected, 0, 1);
static bool network_rejected = false;

static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
    case LTE_LC_EVT_NW_REG_STATUS:
        if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
            (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            LOG_INF("Connected to LTE network!");
            k_sem_give(&lte_connected);
        } else if (evt->nw_reg_status == LTE_LC_NW_REG_REGISTRATION_DENIED) {
            LOG_ERR("Network rejected us!");
            network_rejected = true;
            k_sem_give(&lte_connected);
        }
        break;
    default:
        break;
    }
}

int main(void)
{
    int err;

    LOG_INF("=== LTE Cyclic Connection Test Booting ===");

    const struct device *npm1300_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300));
    if (!device_is_ready(npm1300_dev)) {
        LOG_ERR("PMIC is not ready");
        return -1;
    } else {
        LOG_INF("PMIC initialized successfully.");
    }

    /* Initialize the modem library ONLY ONCE */
    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem init failed, err %d", err);
        return -1;
    }

    /* --- The Infinite Reconnect Loop --- */
    while (1) {
        /* Reset flags and semaphores for a fresh attempt */
        network_rejected = false;
        k_sem_reset(&lte_connected);

        LOG_INF("Connecting to LTE network...");
        err = lte_lc_connect_async(lte_handler);
        if (err) {
            LOG_ERR("LTE connect async failed, err %d", err);
            k_sleep(K_SECONDS(10));
            continue; /* Skip the rest of the loop and try again */
        }

        LOG_INF("Waiting for LTE connection...");
        k_sem_take(&lte_connected, K_FOREVER);

        if (!network_rejected) {
            LOG_INF("Successfully connected! Holding for 10 seconds...");
            k_sleep(K_SECONDS(10));
            
            /* lte_lc_power_off() safely detaches from the tower and turns off the radio */
            LOG_INF("Disconnecting cleanly from LTE network...");
            lte_lc_power_off();
            LOG_INF("Disconnected safely.");
        } else {
            LOG_ERR("Aborting this cycle due to network rejection.");
        }

        /* Wait 30 seconds before attempting to connect again */
        LOG_INF("Sleeping for 30 seconds before the next connection attempt...");
        k_sleep(K_SECONDS(30));
        LOG_INF("--------------------------------------------------");
    }

    return 0;
}
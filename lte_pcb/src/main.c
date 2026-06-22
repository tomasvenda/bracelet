#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>
#include <zephyr/drivers/regulator.h>


LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

static K_SEM_DEFINE(lte_connected, 0, 1);

static void lte_handler(const struct lte_lc_evt *const evt)
{
    switch (evt->type) {
    case LTE_LC_EVT_NW_REG_STATUS:
        if ((evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME) ||
            (evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_ROAMING)) {
            LOG_INF("Connected to LTE network!");
            k_sem_give(&lte_connected);
        } else if (evt->nw_reg_status == LTE_LC_NW_REG_REGISTRATION_DENIED) {
            LOG_ERR("Network rejected us! Turning off modem to prevent FPLMN lock.");
            lte_lc_offline();
            k_sleep(K_MINUTES(2));
            lte_lc_normal();
        }
        break;
    default:
        break;
    }
}

int main(void)
{
    int err;

    LOG_INF("=== Simple LTE Test Booting ===");

	// 
	const struct device *npm1300_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300));
	if (!device_is_ready(npm1300_dev)) {
	    LOG_ERR("PMIC is not ready");
	    return -1;
	} else {
	    LOG_INF("PMIC initialized successfully.");
	}

    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem init failed, err %d", err);
        return -1;
    }

    LOG_INF("Connecting to LTE network...");
    err = lte_lc_connect_async(lte_handler);
    if (err) {
        LOG_ERR("LTE connect async failed, err %d", err);
        return -1;
    }

    /* Wait for LTE to connect */
    LOG_INF("Waiting for LTE connection...");
    k_sem_take(&lte_connected, K_FOREVER);
    LOG_INF("Successfully connected to LTE network!");

    while (1) {
        LOG_INF("LTE connection active. Waiting...");
        k_sleep(K_SECONDS(10));
    }

    return 0;
}
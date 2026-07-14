#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/sensor.h>
#include <modem/nrf_modem_lib.h>
#include <modem/lte_lc.h>

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

    case LTE_LC_EVT_LTE_MODE_UPDATE:
        /* Tells you whether the modem actually picked LTE-M or NB-IoT */
        LOG_INF("Active LTE mode: %s",
                evt->lte_mode == LTE_LC_LTE_MODE_LTEM  ? "LTE-M" :
                evt->lte_mode == LTE_LC_LTE_MODE_NBIOT ? "NB-IoT" : "None");
        break;

    case LTE_LC_EVT_RRC_UPDATE:
        LOG_INF("RRC mode: %s",
                evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
        break;

    default:
        break;
    }
}

/* Log battery voltage from the nPM1300 charger (useful now that we run on battery) */
static void log_battery_voltage(void)
{
    static const struct device *const charger =
        DEVICE_DT_GET(DT_NODELABEL(npm1300_charger));
    struct sensor_value volt;

    if (!device_is_ready(charger)) {
        LOG_WRN("Charger device not ready, skipping battery readout.");
        return;
    }

    if (sensor_sample_fetch(charger) == 0 &&
        sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &volt) == 0) {
        LOG_INF("Battery voltage: %d.%03d V", volt.val1, volt.val2 / 1000);
    } else {
        LOG_WRN("Failed to read battery voltage.");
    }
}

int main(void)
{
    int err;

    LOG_INF("=== LTE Cyclic Connection Test Booting (LTE-M preferred) ===");

    const struct device *const npm1300_dev = DEVICE_DT_GET(DT_NODELABEL(npm1300));
    if (!device_is_ready(npm1300_dev)) {
        LOG_ERR("PMIC is not ready");
        return -1;
    }
    LOG_INF("PMIC initialized successfully.");

    log_battery_voltage();

    /* Initialize the modem library ONLY ONCE */
    LOG_INF("Initializing modem library...");
    err = nrf_modem_lib_init();
    if (err) {
        LOG_ERR("Modem init failed, err %d", err);
        return -1;
    }

    /* --- The Infinite Reconnect Loop --- */
    while (1) {
        network_rejected = false;
        k_sem_reset(&lte_connected);

        LOG_INF("Connecting to LTE network...");
        err = lte_lc_connect_async(lte_handler);
        if (err) {
            LOG_ERR("LTE connect async failed, err %d", err);
            k_sleep(K_SECONDS(10));
            continue;
        }

        LOG_INF("Waiting for LTE connection...");
        k_sem_take(&lte_connected, K_FOREVER);

        if (!network_rejected) {
            LOG_INF("Successfully connected! Holding for 10 seconds...");
            log_battery_voltage();
            k_sleep(K_SECONDS(10));

            LOG_INF("Disconnecting cleanly from LTE network...");
            lte_lc_power_off();
            LOG_INF("Disconnected safely.");
        } else {
            LOG_ERR("Aborting this cycle due to network rejection.");
        }

        LOG_INF("Sleeping for 30 seconds before the next connection attempt...");
        log_battery_voltage();
        k_sleep(K_SECONDS(30));
        LOG_INF("--------------------------------------------------");
    }

    return 0;
}
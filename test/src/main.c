#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/regulator.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bodge_test, LOG_LEVEL_INF);

#define BUCKEN_PIN 17 

static const struct device *gpio0_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static const struct device *ldo1_dev  = DEVICE_DT_GET(DT_NODELABEL(npm1300_ldsw1));

int main(void)
{
    LOG_INF("--- ASSASSINATE AND RESUSCITATE SEQUENCE ---");

    if (!device_is_ready(gpio0_dev) || !device_is_ready(ldo1_dev)) {
        LOG_ERR("Devices not ready.");
        return 0;
    }

    /* STEP 1: PIN BUCKEN TO GROUND */
    gpio_pin_configure(gpio0_dev, BUCKEN_PIN, GPIO_OUTPUT_INACTIVE);
    
    /* STEP 2: KILL THE 3.3V RAIL */
    /* Force LDO1 off to shut down the buck-boost converter */
    regulator_disable(ldo1_dev);
    LOG_INF("1. LDO1 KILLED. The 3.3V LED should turn OFF right now.");

    /* Wait 3 full seconds. Watch the LED. 
       This drains the capacitors and breaks the Wi-Fi chip's latch-up state. */
    k_sleep(K_SECONDS(20));

    /* STEP 3: REBUILD THE POWER CLEANLY */
    LOG_INF("2. Turning 3.3V (LDO1) back ON cleanly.");
    regulator_enable(ldo1_dev);

    /* Give the buck-boost 500ms to stabilize its magnetic field */
    LOG_INF("CHECK 2.6v");
    k_sleep(K_SECONDS(30));

    /* STEP 4: WAKE UP WI-FI CHIP */
    LOG_INF("3. Waking up Wi-Fi chip (BUCKEN = HIGH).");
    gpio_pin_set(gpio0_dev, BUCKEN_PIN, 1);  

    LOG_INF("--- SEQUENCE COMPLETE. MEASURE RAILS NOW ---");

    while (1) {
        k_sleep(K_SECONDS(1)); 
    }

    return 0;
}
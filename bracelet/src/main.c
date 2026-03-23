#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "data_manager.h"

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

int main(void)
{
    LOG_INF("=== Bracelet Tracker Booting ===");
    LOG_INF("System initializing... waiting for movement.");

    struct wifi_msg_t rx_msg;

    /* Main Application Loop */
    while (1) {
        /* * This blocks FOREVER until data arrives in the queue.
         * The MCU main thread sleeps completely while waiting.
         */
        k_msgq_get(&wifi_data_queue, &rx_msg, K_FOREVER);

        /* * In the future, this is where we will format the JSON string 
         * and call our MQTT publish function! For now, we mock it.
         */
        LOG_INF("[MOCK MQTT PUBLISH] Sending -> SSID: %-15s | MAC: %s | RSSI: %d", 
                rx_msg.ssid, rx_msg.mac, rx_msg.rssi);
    }

    return 0;
}
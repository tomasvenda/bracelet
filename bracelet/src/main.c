#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <string.h>

#include "data_manager.h"
#include "lte_service.h" 

LOG_MODULE_REGISTER(main_app, LOG_LEVEL_INF);

extern int low_power_acl_init(void);

int main(void)
{
    LOG_INF("=== Bracelet Tracker Booting ===");
    
    if (lte_mqtt_init() != 0) {
        LOG_ERR("LTE/MQTT Initialization failed! Halting.");
        return -1;
    }

    if (low_power_acl_init() != 0) {
        LOG_ERR("Interrupt setup failed!");
    } else {
        LOG_INF("Interrupt setup SUCCESSFUL! Waiting for movement...");
    }

    struct wifi_batch_msg_t rx_msg;
    char json_payload[1024]; 

    while (1) {
        if (k_msgq_get(&wifi_data_queue, &rx_msg, K_FOREVER) == 0) {
            
            /* Start building the JSON payload in nRF Cloud "wifi" format */
            int offset = snprintf(json_payload, sizeof(json_payload), 
                                  "{\"wifi\":{\"accessPoints\":[");

            for (int i = 0; i < rx_msg.ap_count; i++) {
                const char *comma = (i == rx_msg.ap_count - 1) ? "" : ",";

                /* nRF Cloud uses 'macAddress' and 'signalStrength' */
                offset += snprintf(json_payload + offset, sizeof(json_payload) - offset,
                                   "{\"macAddress\":\"%s\",\"signalStrength\":%d}%s",
                                   rx_msg.aps[i].mac, rx_msg.aps[i].rssi, comma);
            }

            /* Close the JSON structure */
            snprintf(json_payload + offset, sizeof(json_payload) - offset, "]}}");

            LOG_INF("Publishing nRF Cloud Format (%d bytes)", strlen(json_payload));
            
            lte_mqtt_publish_str(json_payload);
        }
    }
    return 0;
}
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

    struct location_msg_t rx_msg;
    char json_payload[1024]; 

    while (1) {
        if (k_msgq_get(&location_queue, &rx_msg, K_FOREVER) == 0) {
            memset(json_payload, 0, sizeof(json_payload));
            
            if (rx_msg.type == LOC_TYPE_WIFI) {
                int offset = snprintf(json_payload, sizeof(json_payload), "{\"wifi\":{\"accessPoints\":[");
                for (int i = 0; i < rx_msg.data.wifi.count; i++) {
                    const char *comma = (i == rx_msg.data.wifi.count - 1) ? "" : ",";
                    offset += snprintf(json_payload + offset, sizeof(json_payload) - offset,
                                       "{\"macAddress\":\"%s\",\"signalStrength\":%d}%s",
                                       rx_msg.data.wifi.aps[i].mac, rx_msg.data.wifi.aps[i].rssi, comma);
                }
                snprintf(json_payload + offset, sizeof(json_payload) - offset, "]}}");
            } 
            else if (rx_msg.type == LOC_TYPE_GNSS) {
                snprintf(json_payload, sizeof(json_payload), 
                         "{\"gnss\":{\"lat\":%.6f,\"lng\":%.6f,\"acc\":%.1f}}",
                         rx_msg.data.gnss.lat, rx_msg.data.gnss.lon, rx_msg.data.gnss.accuracy);
            }
            else if (rx_msg.type == LOC_TYPE_CELL) {
                snprintf(json_payload, sizeof(json_payload),
                         "{\"lte\":{\"mcc\":%d,\"mnc\":%d,\"cellId\":%d,\"areaCode\":%d}}",
                         rx_msg.data.cell.mcc, rx_msg.data.cell.mnc, 
                         rx_msg.data.cell.cell_id, rx_msg.data.cell.area_code);
            }

            LOG_INF("Publishing Location Payload: %s", json_payload);
            lte_mqtt_publish_str(json_payload);
        }
    }
    return 0;
}
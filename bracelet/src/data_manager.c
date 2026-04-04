#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "data_manager.h"

LOG_MODULE_REGISTER(data_manager, LOG_LEVEL_INF);

/* Generic location queue for Wi-Fi, GNSS, or Cell data */
K_MSGQ_DEFINE(location_queue, sizeof(struct location_msg_t), 5, 4);

void data_manager_send_wifi(struct ap_data_t *aps, uint8_t count)
{
    struct location_msg_t msg;

    msg.type = LOC_TYPE_WIFI;
    msg.data.wifi.count = count;

    for (int i = 0; i < count; i++) {
        strncpy(msg.data.wifi.aps[i].mac, aps[i].mac, sizeof(msg.data.wifi.aps[i].mac) - 1);
        msg.data.wifi.aps[i].mac[sizeof(msg.data.wifi.aps[i].mac) - 1] = '\0';
        msg.data.wifi.aps[i].rssi = aps[i].rssi;
    }

    if (k_msgq_put(&location_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Location queue full! Dropping Wi-Fi data.");
    }
}

void data_manager_send_gnss(double lat, double lon, float accuracy)
{
    struct location_msg_t msg;
    
    msg.type = LOC_TYPE_GNSS;
    msg.data.gnss.lat = lat;
    msg.data.gnss.lon = lon;
    msg.data.gnss.accuracy = accuracy;

    if (k_msgq_put(&location_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Location queue full! Dropping GNSS data.");
    }
}

void data_manager_send_cell(uint16_t mcc, uint16_t mnc, uint32_t cell_id, uint16_t area_code)
{
    struct location_msg_t msg;

    msg.type = LOC_TYPE_CELL;
    msg.data.cell.mcc = mcc;
    msg.data.cell.mnc = mnc;
    msg.data.cell.cell_id = cell_id;
    msg.data.cell.area_code = area_code;

    if (k_msgq_put(&location_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("Location queue full! Dropping Cell data.");
    }
}
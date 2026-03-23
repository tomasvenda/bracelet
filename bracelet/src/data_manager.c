#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "data_manager.h"

LOG_MODULE_REGISTER(data_manager, LOG_LEVEL_INF);

/* Create a mailbox that can hold up to 20 WiFi scan results at a time */
K_MSGQ_DEFINE(wifi_data_queue, sizeof(struct wifi_msg_t), 20, 4);

void data_manager_send_wifi_result(const char *ssid, int rssi, const char *mac)
{
    struct wifi_msg_t msg;

    /* Safely copy the data into our message structure */
    strncpy(msg.ssid, ssid, sizeof(msg.ssid) - 1);
    msg.ssid[sizeof(msg.ssid) - 1] = '\0';
    
    strncpy(msg.mac, mac, sizeof(msg.mac) - 1);
    msg.mac[sizeof(msg.mac) - 1] = '\0';

    msg.rssi = rssi;

    /* Put the message into the queue. 
     * K_NO_WAIT means if the queue is perfectly full (20 items), 
     * we just drop this result rather than freezing the WiFi thread. 
     */
    if (k_msgq_put(&wifi_data_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("WiFi data queue is full! Dropping result.");
    }
}
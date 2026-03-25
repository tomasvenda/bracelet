#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "data_manager.h"

LOG_MODULE_REGISTER(data_manager, LOG_LEVEL_INF);

/* Mailbox for our batches. We only need space for a few since they are processed instantly */
K_MSGQ_DEFINE(wifi_data_queue, sizeof(struct wifi_batch_msg_t), 5, 4);

void data_manager_send_wifi_batch(struct ap_data_t *aps, uint8_t count)
{
    struct wifi_batch_msg_t msg;
    
    msg.ap_count = count;
    
    /* Copy the array of APs into our message */
    for (int i = 0; i < count; i++) {
        strncpy(msg.aps[i].mac, aps[i].mac, sizeof(msg.aps[i].mac) - 1);
        msg.aps[i].mac[sizeof(msg.aps[i].mac) - 1] = '\0';
        msg.aps[i].rssi = aps[i].rssi;
    }   

    /* Ship the batch to main.c */
    if (k_msgq_put(&wifi_data_queue, &msg, K_NO_WAIT) != 0) {
        LOG_WRN("WiFi data queue is full! Dropping batch.");
    }
}
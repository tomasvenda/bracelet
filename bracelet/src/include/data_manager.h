#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <stdint.h>
#include <zephyr/kernel.h>

#define MAX_APS_PER_SCAN 10

/* Structure for a single Access Point */
struct ap_data_t {
    char mac[18];
    int rssi;
};

/* The new Batch Message Structure */
struct wifi_batch_msg_t {
    uint8_t ap_count;
    struct ap_data_t aps[MAX_APS_PER_SCAN];
};

extern struct k_msgq wifi_data_queue;

/* New function signature to send the whole batch at once */
void data_manager_send_wifi_batch(struct ap_data_t *aps, uint8_t count);

#endif
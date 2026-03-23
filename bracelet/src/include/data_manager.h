#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <zephyr/kernel.h>

/* The structure of the message we will send to the main thread */
struct wifi_msg_t {
    char ssid[33];   /* Max SSID length is 32 + null terminator */
    char mac[18];    /* MAC address string "xx:xx:xx:xx:xx:xx" + null */
    int rssi;
};

/* The Zephyr Message Queue (Mailbox) */
extern struct k_msgq wifi_data_queue;

/* Function for the WiFi thread to push data into the queue */
void data_manager_send_wifi_result(const char *ssid, int rssi, const char *mac);

#endif /* DATA_MANAGER_H */
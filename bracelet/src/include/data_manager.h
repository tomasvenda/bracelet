#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#include <stdint.h>
#include <zephyr/kernel.h>

#define MAX_APS_PER_SCAN 10

enum location_type {
    LOC_TYPE_WIFI,
    LOC_TYPE_GNSS,
    LOC_TYPE_CELL
};

/* Structure for a single Access Point */
struct ap_data_t {
    char mac[18];
    int rssi;
};

struct wifi_data_t {
    uint8_t count;
    struct ap_data_t aps[MAX_APS_PER_SCAN];
};

struct gnss_data_t {
    double lat;
    double lon;
    float accuracy;
};

struct cell_data_t {
    uint16_t mcc;
    uint16_t mnc;
    uint32_t cell_id;
    uint16_t area_code;
};

/* The Generic Location Message Structure */
struct location_msg_t {
    enum location_type type;
    union {
        struct wifi_data_t wifi;
        struct gnss_data_t gnss;
        struct cell_data_t cell;
    } data;
};

extern struct k_msgq location_queue;

/* Functions to send different types of localization data */
void data_manager_send_wifi(struct ap_data_t *aps, uint8_t count);
void data_manager_send_gnss(double lat, double lon, float accuracy);
void data_manager_send_cell(uint16_t mcc, uint16_t mnc, uint32_t cell_id, uint16_t area_code);

#endif
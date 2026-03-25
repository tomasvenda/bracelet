#ifndef LTE_SERVICE_H
#define LTE_SERVICE_H

#include <stdint.h>

/* Initializes the modem, connects to LTE, and connects to Azure MQTT */
int lte_mqtt_init(void);

/* Publishes a raw string payload to the Azure MQTT broker */
int lte_mqtt_publish_str(const char *payload);

#endif /* LTE_SERVICE_H */
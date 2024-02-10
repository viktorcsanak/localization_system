#pragma once

#include <stdint.h>

typedef enum rtls_role {
    UNCONFIGURED = 0,
    TAG,
    ANCHOR,
    GATEWAY_ANCHOR,
} rtls_role_t;

struct device_config {
    uint32_t device_id;
    uint8_t rtls_role;
    uint16_t adv_duration;
};

int dev_mgmt_set_rtls_role(struct device_config *config, rtls_role_t rtls_role);
int dev_mgmt_set_adv_duration(struct device_config *config, uint16_t duration);

int dev_mgmt_config_load(struct device_config *config);
int dev_mgmt_config_save(struct device_config *config);

struct device_config *dev_mgmt_get_config(void);
void dev_mgmt_set_config(struct device_config *config);

int dev_mgmt_config_init(void);

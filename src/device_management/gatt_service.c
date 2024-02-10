#include "include/device_management.h"
#include "include/gatt_service.h"
#include "include/device_config.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gatt_service, CONFIG_LOG_DEFAULT_LEVEL);

static struct device_config config;

static ssize_t read_u32(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                void *buf, uint16_t len, uint16_t offset) {
    const uint32_t *p_user_data = (uint32_t *)attr->user_data;

    uint32_t value = sys_cpu_to_le32(*p_user_data);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

static ssize_t read_u16(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                void *buf, uint16_t len, uint16_t offset) {
    const uint16_t *p_user_data = (uint16_t *)attr->user_data;

    uint16_t value = sys_cpu_to_le16(*p_user_data);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

static ssize_t write_role(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uint8_t rtls_role;
    memcpy(&rtls_role + offset, buf, len);
    int ret = dev_mgmt_set_rtls_role(&config, rtls_role);
    if (ret) {
        return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
    }

    return len;
}

static ssize_t write_adv_duration(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    uint16_t adv_duration;
    memcpy(&adv_duration + offset, buf, len);
    int ret = dev_mgmt_set_adv_duration(&config, adv_duration);
    if (ret) {
        return BT_GATT_ERR(BT_ATT_ERR_OUT_OF_RANGE);
    }

    return len;
}

static ssize_t save_current_config(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                            const void *buf, uint16_t len, uint16_t offset, uint8_t flags) {
    int ret = dev_mgmt_config_save(&config);
    if (ret) {
        return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
    }

    return len;
}

BT_GATT_SERVICE_DEFINE(dev_mgmt_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DEV_MGMT_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_DEV_MGMT_DEVICE_ID,
                    BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_u32, NULL, &config.device_id),
    BT_GATT_CHARACTERISTIC(BT_UUID_DEV_MGMT_DEVICE_ROLE,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                    read_u16, write_role, &config.rtls_role),
    BT_GATT_CHARACTERISTIC(BT_UUID_DEV_MGMT_ADV_DURATION,
                    BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
                    read_u16, write_adv_duration, &config.adv_duration),
    BT_GATT_CHARACTERISTIC(BT_UUID_DEV_MGMT_CONFIGURATION_SAVE,
                    BT_GATT_CHRC_WRITE,
                    BT_GATT_PERM_WRITE,
                    NULL, save_current_config, NULL),
);

int dev_mgmt_gatt_service_init(void) {
    int ret = dev_mgmt_config_load(&config);
    if (ret) {
        LOG_ERR("%s: failed to load configurations %d", __func__, ret);
        return ret;
    }

    dev_mgmt_set_config(&config);
    return 0;
}

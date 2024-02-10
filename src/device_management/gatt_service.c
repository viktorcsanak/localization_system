#include "include/device_management.h"
#include "include/gatt_service.h"

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/bluetooth/gatt.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(gatt_service, CONFIG_LOG_DEFAULT_LEVEL);

static uint32_t role = 0;

static ssize_t read_u32(struct bt_conn *conn, const struct bt_gatt_attr *attr,
                void *buf, uint16_t len, uint16_t offset) {
    const uint32_t *p_user_data = (uint32_t *)attr->user_data;

    uint32_t value = sys_cpu_to_le16(*p_user_data);

    return bt_gatt_attr_read(conn, attr, buf, len, offset, &value, sizeof(value));
}

BT_GATT_SERVICE_DEFINE(rtls_service,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_RTLS_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_RTLS_DEVICE_ROLE,
                    BT_GATT_CHRC_READ,
                    BT_GATT_PERM_READ,
                    read_u32, NULL, &role),
);

int dev_mgmt_gatt_service_init(void) {
    return 0;
}

#include "include/device_management.h"
#include "include/gatt_service.h"
#include "include/device_config.h"

#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>

#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(device_management, CONFIG_LOG_DEFAULT_LEVEL);

#define DEVICE_MANAGER_STACK_SIZE 2048
#define DEVICE_MANAGER_PRIORITY 0

#define BLE_WAKE_UP_NODE DT_ALIAS(sw0)

static const struct gpio_dt_spec ble_wake_up = GPIO_DT_SPEC_GET(BLE_WAKE_UP_NODE, gpios);

K_THREAD_STACK_DEFINE(device_manager_stack_area, DEVICE_MANAGER_STACK_SIZE);
struct k_work_q dev_mgmt_work_q;

struct bt_conn *default_conn;

static const struct bt_data advertise_data[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, DEV_MGMT_UUID_SERVICE, DEV_MGMT_UUID_BASE),
};

static void adv_stop_worker(struct k_work *item) {
    k_free(item);

    dev_mgmt_bt_stop_advertising();
}

static void advertisement_expiry_callback(struct k_timer *timer) {
    struct k_work *adv_stop_work = (struct k_work *)k_calloc(1, sizeof(struct k_work));

    if (adv_stop_work == NULL) {
        LOG_ERR("%s: failed to allocate work item", __func__);
        return;
    }

    k_work_init(adv_stop_work, adv_stop_worker);

    if (k_work_submit_to_queue(&dev_mgmt_work_q, adv_stop_work) <= 0) {
        k_free(adv_stop_work);
    }
}

K_TIMER_DEFINE(advertisement_timer, advertisement_expiry_callback, NULL);

/** @brief Wrapper to start adveritising with required data
 * @retval 0 Success
 * @retval -ERRNO Failed
*/
int dev_mgmt_bt_start_advertising(void) {
    uint32_t unique_id = dev_mgmt_get_config()->device_id;
    if (unique_id < 0) {
        LOG_ERR("%s: Failed to read device_id: %d", __func__, unique_id);
        return unique_id;
    }

    static struct bt_data scan_data[] = {0};
    static uint8_t device_data[8 + (2 * sizeof(uint32_t))] = {0};
    uint8_t data_len = ARRAY_SIZE(device_data);

    scan_data->type = BT_DATA_NAME_COMPLETE;
    scan_data->data_len = data_len;
    memset(&device_data[0], data_len, 0);
    sprintf(&device_data[0], "Anchor_%8x", unique_id);
    scan_data->data = &device_data[0];

    int rc = bt_le_adv_start(
        BT_LE_ADV_CONN,
        &advertise_data[0], ARRAY_SIZE(advertise_data),
        &scan_data[0], ARRAY_SIZE(scan_data));

    if (rc) {
        LOG_ERR("%s: failed to start advertising: %d", __func__, rc);
        return rc;
    }

    const rtls_role_t role = dev_mgmt_get_config()->rtls_role;
    if (role != GATEWAY_ANCHOR) {
        const uint16_t adv_duration = dev_mgmt_get_config()->adv_duration;
        k_timer_start(&advertisement_timer, K_SECONDS(adv_duration), K_NO_WAIT);
        LOG_INF("%s: start advertising for %d seconds", __func__, adv_duration);
    }
    else {
        LOG_INF("%s: start advertising indefinitely", __func__);
    }

    return 0;
}

/** @brief Wrapper to stop adveritising with required data
 * @retval 0 Success
 * @retval -ERRNO Failed
*/
int dev_mgmt_bt_stop_advertising(void) {
    int rc = bt_le_adv_stop();

    if (rc) {
        LOG_ERR("%s: failed to stop advertising: %d", __func__, rc);
        return rc;
    }

    LOG_INF("%s: stop advertising", __func__);
    return 0;
}

static void connected_cb(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_WRN("%s: connection failed: %d", __func__, err);
        return;
    }

    default_conn = bt_conn_ref(conn);
    LOG_INF("%s: Bluetooth connected", __func__);
}

static void disconnected_cb(struct bt_conn * conn, uint8_t reason)
{
    LOG_INF("%s: Disconnected: %u\n", __func__, reason);

    if (default_conn) {
        bt_conn_unref(default_conn);
        default_conn = NULL;
    }
}

static struct bt_conn_cb connection_callbacks = {
    .connected    = connected_cb,
    .disconnected = disconnected_cb,
};

/** @brief Check BT connection status
 * @retval 0 not connected
 * @retval 1 connected
 * @retval -ERRNO failed to get status
*/
int dev_mgmt_bt_is_connected(void) {
	struct bt_conn_info info;

    int rc = bt_conn_get_info(default_conn, &info);
    if (rc) {
        LOG_ERR("%s: failed to get connection info: %d", __func__, rc);
        return rc;
    }

    return (info.state == BT_CONN_STATE_CONNECTED ? 1 : 0);
}

int dev_mgmt_bt_disconnect_peer(void) {
    if (!default_conn) {
        return -ENOMEM;
    }

    int rc = dev_mgmt_bt_is_connected();
    if (rc <= 0) {
        return rc;
    }

    struct bt_conn *conn = default_conn;

    bt_conn_unref(default_conn);
    default_conn = NULL;

    k_msleep(50);

    rc = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);

    return rc;
}

static void auth_cancel(struct bt_conn *conn)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

    LOG_INF("%s: pairing cancelled: %s\n", __func__, addr);
}

static struct bt_conn_auth_cb  auth_cb_display = {
    .cancel = auth_cancel,
};

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("%s: bluetooth initialization failed: %d\n", __func__, err);
        return;
    }

    LOG_INF("%s: bluetooth initialized OK", __func__);
}

static void adv_start_worker(struct k_work *item) {
    k_free(item);

    dev_mgmt_bt_start_advertising();
}

static void ble_wake(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
    struct k_work *adv_start_work = (struct k_work *)k_calloc(1, sizeof(struct k_work));

    if (adv_start_work == NULL) {
        LOG_ERR("%s: failed to allocate work item", __func__);
        return;
    }

    k_work_init(adv_start_work, adv_start_worker);

    if (k_work_submit_to_queue(&dev_mgmt_work_q, adv_start_work) <= 0) {
        k_free(adv_start_work);
    }
}

static struct gpio_callback ble_wake_cb_data;

static int gpio_setup(void) {
    if (!gpio_is_ready_dt(&ble_wake_up)) {
        LOG_ERR("%s: ble wake button is not ready", __func__);
        return -ENODEV;
    }

    int rc = gpio_pin_configure_dt(&ble_wake_up, GPIO_INPUT);
    if (rc) {
        LOG_ERR("%s: failed to configure ble wake button", __func__);
        return -EINVAL;
    }

    rc = gpio_pin_interrupt_configure_dt(&ble_wake_up, GPIO_INT_EDGE_TO_ACTIVE);
    if (rc) {
        LOG_ERR("%s: failed to configure interrupt on ble wake button", __func__);
        return -EINVAL;
    }

    gpio_init_callback(&ble_wake_cb_data, ble_wake, BIT(ble_wake_up.pin));
    gpio_add_callback_dt(&ble_wake_up, &ble_wake_cb_data);

    return 0;
}

int dev_mgmt_init(void) {
    int rc = bt_enable(bt_ready);
    if (rc) {
        LOG_ERR("%s: bluetooth initialization failed: %d\n", __func__, rc);
        return rc;
    }

    bt_conn_cb_register(&connection_callbacks);
    rc = bt_conn_auth_cb_register(&auth_cb_display);
    if (rc) {
        LOG_ERR("%s: failed to register bt auth callbacks %d", __func__, rc);
    }

    k_work_queue_init(&dev_mgmt_work_q);
    k_work_queue_start(&dev_mgmt_work_q, device_manager_stack_area,
                        K_THREAD_STACK_SIZEOF(device_manager_stack_area), DEVICE_MANAGER_PRIORITY, NULL);

    k_timer_init(&advertisement_timer, advertisement_expiry_callback, NULL);

    rc = gpio_setup();
    if (rc) {
        LOG_ERR("%s: failed to setup GPIOs %d", __func__, rc);
    }

    return 0;
}

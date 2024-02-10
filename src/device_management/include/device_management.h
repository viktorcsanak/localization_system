#pragma once

#include "gatt_uuids.h"

int dev_mgmt_bt_start_advertising(void);
int dev_mgmt_bt_stop_advertising(void);

int dev_mgmt_bt_is_connected(void);
int dev_mgmt_bt_disconnect_peer(void);

int dev_mgmt_init(void);

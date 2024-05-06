#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <deca_types.h>
#include <port.h>

#include <device_management.h>
#include <gatt_service.h>
#include <device_config.h>

#include <radio_handler.h>

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

extern int dw_twr_resp_main(void);
extern int dw_twr_init_main(void);

int main(void)
{
    int rc = dev_mgmt_config_init();
    if (rc) {
        LOG_ERR("%s: failed to load configuration %d", __func__, rc);
        return rc;
    }

    rc = dev_mgmt_init();
    if (rc) {
        LOG_ERR("%s: failed to initialize device management %d", __func__, rc);
        return rc;
    }

    rc = dev_mgmt_gatt_service_init();

    rc = radio_handler_init();
    if (rc) {
        LOG_ERR("%s: failed to initialize radio handler %d", __func__, rc);
        return rc;
    }

    //dw_twr_init_main();
    //dw_twr_resp_main();

    uint8_t rtls_role = dev_mgmt_get_config()->rtls_role;
    uint32_t device_id = dev_mgmt_get_config()->device_id;
    LOG_INF("Device %08x booting as %d", device_id, rtls_role);

    if (rtls_role != TAG) {
        return 0;
    }

    while (1) {
        rc = start_measurement();
        if (rc) {
            LOG_ERR("%s: failed to start measurement %d", __func__, rc);
        }
        k_sleep(K_MSEC(1000));
    }

    return -1;
}

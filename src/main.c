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
    LOG_INF("Device booting as %d", rtls_role);
    if (rtls_role == GATEWAY_ANCHOR) {
        uint32_t target_id = 0x444a8ca3;
        while (1) {
            rc = start_measurement(target_id);
            if (rc) {
                LOG_ERR("%s: failed to start measurement %d", __func__, rc);
            }
            k_sleep(K_MSEC(1000));
        }
    }

    return 0;
}

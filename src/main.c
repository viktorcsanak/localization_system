#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <deca_types.h>
#include <port.h>

#include <device_management.h>
#include <gatt_service.h>
#include <device_config.h>

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

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

    return 0;
}

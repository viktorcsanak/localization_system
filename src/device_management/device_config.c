#include "include/device_config.h"

#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>

#include <zephyr/drivers/hwinfo.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(device_config, CONFIG_LOG_DEFAULT_LEVEL);

#define DEVICE_ID_MAX_LENGTH 8
#define FNV_PRIME           16777619U
#define FNV_OFFSET_BASIS    2166136261U

#define DEV_CONF_RTLS_ROLE_CFG_ID 0
#define DEV_CONF_ADV_DURATION_CFG_ID 1

#define DEV_CONF_RTLS_ROLE_DEFAULT_VALUE 0
#define DEV_CONF_ADV_DURATION_DEFAULT_VALUE 20

#define ADV_DURATION_LO_LIMIT DEV_CONF_ADV_DURATION_DEFAULT_VALUE
#define ADV_DURATION_HI_LIMIT 600

#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(storage_partition)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(storage_partition)

static struct device_config *_config;

static struct nvs_fs file_system = {
    .flash_device   = NVS_PARTITION_DEVICE,
    .offset         = NVS_PARTITION_OFFSET,
};

struct configuration_member {
    uint16_t member_offset;
    uint16_t member_size;
    uint8_t nvs_id;
    int32_t default_value;
};

static const struct configuration_member config_map[] = {
    {offsetof(struct device_config, rtls_role), sizeof(uint8_t),
        DEV_CONF_RTLS_ROLE_CFG_ID, DEV_CONF_RTLS_ROLE_DEFAULT_VALUE},
    {offsetof(struct device_config, adv_duration), sizeof(uint16_t),
        DEV_CONF_ADV_DURATION_CFG_ID, DEV_CONF_ADV_DURATION_DEFAULT_VALUE},
};

typedef enum storage_op {
    load_config = 0,
    save_config,
} storage_op_t;

static int storage_iterator_perform(struct device_config *config, storage_op_t operation) {
    uint8_t n_keys = ARRAY_SIZE(config_map);

    int byte_count = 0;
    int again = 0;
    for (int i = 0; i < n_keys; i++) {
        struct configuration_member configuration = config_map[i];
        void *data = (uint8_t *)config + configuration.member_offset;

        int rc;
        //Check for flash operations
        if (operation == load_config) {
            rc = nvs_read(&file_system, configuration.nvs_id, data, configuration.member_size);
        }
        else if (operation == save_config) {
            rc = nvs_write(&file_system, configuration.nvs_id, data, configuration.member_size);
        }

        //Write default values if operation failed with no entry error
        if (-rc == ENOENT) {
            LOG_INF("%s: Writing default value %d of key ID %d", __func__, configuration.default_value, configuration.nvs_id);
            rc = nvs_write(&file_system, configuration.nvs_id, &configuration.default_value, configuration.member_size);
            again = EAGAIN;
        }

        //Always evaluate this condition so we can check the result of writing the default value
        if(rc < 0) {
            LOG_ERR("%s: failed to read/write configuration key_id %d with error %s", __func__, i, strerror(-rc));
            return rc;
        }

        byte_count += rc;
    }

    return (again == EAGAIN) ? -EAGAIN : byte_count;
}

static int fs_init(void) {
    if (!device_is_ready(file_system.flash_device)) {
        LOG_ERR("%s: device is not ready", __func__);
        return -ENODEV;
    }

    struct flash_pages_info info;
    int ret = flash_get_page_info_by_offs(file_system.flash_device, file_system.offset, &info);

    if (ret) {
        LOG_ERR("%s: unable to get page info", __func__);
        return ret;
    }

    file_system.sector_size     = info.size;
    file_system.sector_count    = 4u;

    ret = nvs_mount(&file_system);

    return 0;
}

static uint32_t fnv_hash(const uint8_t *data, uint8_t data_length) {
    uint32_t hash = FNV_OFFSET_BASIS;
    for (int i = 0; i < data_length; i++) {
        hash ^= data[i];
        hash *= FNV_PRIME;
    }

    return hash;
}

static uint32_t get_device_id(void) {
    uint8_t device_id[DEVICE_ID_MAX_LENGTH] = {0};
    int rc = hwinfo_get_device_id(&device_id[0], DEVICE_ID_MAX_LENGTH);

    if (rc <= 0) {
        LOG_ERR("%s: reading device ID failed or length is 0: %d", __func__, -rc);
        return 0;
    }

    return fnv_hash(device_id, DEVICE_ID_MAX_LENGTH);
}

static int configuration_load(struct device_config *config) {
    config->device_id = get_device_id();

    int ret = storage_iterator_perform(config, load_config);
    if (-ret == EAGAIN) {
        ret = storage_iterator_perform(config, load_config);
    }
    if (ret < 0) {
        LOG_ERR("%s: failed to load from storage %d", __func__, ret);
        return ret;
    }

    return 0;
}

static int configuration_save(struct device_config *config) {
    return storage_iterator_perform(config, save_config);
}

int dev_mgmt_set_rtls_role(struct device_config *config, rtls_role_t rtls_role) {
    uint8_t valid_value = (rtls_role == ANCHOR) || (rtls_role == TAG) || (rtls_role == GATEWAY_ANCHOR);
    if (!valid_value) {
        LOG_ERR("%s: requested invalid value of %d", __func__, rtls_role);
        return -EINVAL;
    }
    config->rtls_role = rtls_role;
    return 0;
}

int dev_mgmt_set_adv_duration(struct device_config *config, uint16_t duration) {
    uint8_t valid_value = (duration >= ADV_DURATION_LO_LIMIT) && (duration <= ADV_DURATION_HI_LIMIT);
    if (!valid_value) {
        LOG_ERR("%s: requested invalid value of %d", __func__, duration);
        return -EINVAL;
    }
    config->adv_duration = duration;
    return 0;
}

int dev_mgmt_config_load(struct device_config *config) {
    return configuration_load(config);
}

int dev_mgmt_config_save(struct device_config *config) {
    return configuration_save(config);
}

void dev_mgmt_set_config(struct device_config *config) {
    _config = config;
}

struct device_config *dev_mgmt_get_config(void) {
    return _config;
}

int dev_mgmt_config_init(void) {
    int ret = fs_init();
    if (ret) {
        LOG_ERR("%s: failed to init nvs_fs %d", __func__, ret);
        return ret;
    }

    return 0;
}

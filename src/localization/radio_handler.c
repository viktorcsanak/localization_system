#include "include/radio_handler.h"

#include <device_config.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>

#include <sys/errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(radio_handler, CONFIG_LOG_DEFAULT_LEVEL);

#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define FRAME_LENGTH_MAX 127

/** @brief {'M', 'R', target_id[4], gateway_id[4], reserved[4], checksum[2]}*/
#define MEAS_REQ_FRAME_LENGTH 16
/** @brief {'M', 'I', target_id[4], gateway_id[4], initiator_id[4], checksum[2]}*/
#define MEAS_INIT_FRAME_LENGTH 16

static struct k_work isr_work;

dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_EXT, /* PHY header mode. */
    (129)            /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static void isr_wrapper(void) {
    k_work_submit(&isr_work);
}

static void isr_handler(struct k_work *item) {
    dwt_isr();
}

struct tx_data {
    uint8_t tx_buffer[FRAME_LENGTH_MAX];
    uint8_t data_length;
};

struct tx_container {
    struct k_work tx_work;
    struct tx_data tx_data;
};

struct tx_schedule_container {
    struct k_work_delayable tx_work;
    struct tx_data tx_data;
};

static uint32_t device_id_sleep(void) {
    uint32_t device_id = dev_mgmt_get_config()->device_id;
    return ((device_id % 983) + 10);
}

static void tx_start(struct tx_data *tx_data) {
    dwt_forcetrxoff();
    
    dwt_writetxdata(tx_data->data_length, &tx_data->tx_buffer[0], 0);
    
    dwt_writetxfctrl(tx_data->data_length, 0, 0);
    
    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

static void tx_immediate(struct k_work *item) {
    struct tx_container *container = CONTAINER_OF(item, struct tx_container, tx_work);
    struct tx_data *tx_data = &container->tx_data;
    
    tx_start(tx_data);

    k_free(container);
}

static void tx_schedule(struct k_work *item) {
    struct tx_schedule_container *container = CONTAINER_OF(k_work_delayable_from_work(item), struct tx_schedule_container, tx_work);
    struct tx_data *tx_data = &container->tx_data;
    
    tx_start(tx_data);

    k_free(container);
}

static int send_measurement_request(const uint32_t target_id) {
    struct tx_container *container = (struct tx_container *)k_calloc(1, sizeof(struct tx_container));
    if (container == NULL) {
        LOG_ERR("%s: failed to allocate container", __func__);
        return -ENOMEM;
    }

    struct tx_data *tx_data = &container->tx_data;
    tx_data->data_length = MEAS_REQ_FRAME_LENGTH;
    memset(&tx_data->tx_buffer[0], 0, MEAS_REQ_FRAME_LENGTH);

    tx_data->tx_buffer[0] = 'M';
    tx_data->tx_buffer[1] = 'R';

    uint32_t gateway_id = dev_mgmt_get_config()->device_id;
    memcpy(&tx_data->tx_buffer[2], &target_id, sizeof(target_id));
    memcpy(&tx_data->tx_buffer[6], &gateway_id, sizeof(gateway_id));

    k_work_init(&container->tx_work, tx_immediate);
    int ret = k_work_submit(&container->tx_work);
    if (ret < 0) {
        LOG_ERR("%s: failed to submit work item %d", __func__, ret);
        return ret;
    }

    return 0;
}

static int send_measurement_init(const uint32_t target_id, const uint32_t gateway_id) {
    struct tx_schedule_container *container = (struct tx_schedule_container *)k_calloc(1, sizeof(struct tx_schedule_container));
    if (container == NULL) {
        LOG_ERR("%s: failed to allocate container", __func__);
        return -ENOMEM;
    }

    struct tx_data *tx_data = &container->tx_data;
    tx_data->data_length = MEAS_INIT_FRAME_LENGTH;
    memset(&tx_data->tx_buffer[0], 0, MEAS_INIT_FRAME_LENGTH);

    tx_data->tx_buffer[0] = 'M';
    tx_data->tx_buffer[1] = 'I';

    uint32_t initiator_id = dev_mgmt_get_config()->device_id;
    memcpy(&tx_data->tx_buffer[2], &target_id, sizeof(target_id));
    memcpy(&tx_data->tx_buffer[6], &gateway_id, sizeof(gateway_id));
    memcpy(&tx_data->tx_buffer[10], &initiator_id, sizeof(initiator_id));

    uint32_t delay = device_id_sleep();
    k_work_init_delayable(&container->tx_work, tx_schedule);
    int ret = k_work_schedule(&container->tx_work, K_MSEC(delay));
    if (ret < 0) {
        LOG_ERR("%s: failed to schedule work item %d", __func__, ret);
        return ret;
    }

    return 0;
}

static int handle_meas_init(uint8_t *frame, uint16_t frame_length) {
    if (MEAS_INIT_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, MEAS_INIT_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }

    uint32_t target_id = 0;
    uint32_t gateway_id = 0;
    uint32_t initiator_id = 0;
    uint32_t device_id = dev_mgmt_get_config()->device_id;
    memcpy(&target_id, &frame[2], sizeof(target_id));
    if (target_id != device_id) {
        LOG_INF("%s: received mes_init was not addressed to this device %08x vs %08x", __func__, target_id, device_id);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -ECANCELED; 
    }
    memcpy(&gateway_id, &frame[6], sizeof(gateway_id));
    memcpy(&initiator_id, &frame[10], sizeof(initiator_id));

    LOG_INF("%s: received meas_init from %08x requested by %08x", __func__, initiator_id, gateway_id);

    return 0;
}

static int handle_meas_req(uint8_t *frame, uint16_t frame_length) {
    if (MEAS_REQ_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, MEAS_REQ_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }

    uint32_t target_id = 0, gateway_id = 0;
    memcpy(&target_id, &frame[2], sizeof(target_id));
    memcpy(&gateway_id, &frame[6], sizeof(gateway_id));
    return send_measurement_init(target_id, gateway_id);
}

static int handle_frame(uint8_t *frame, uint16_t frame_length) {
    rtls_role_t role = dev_mgmt_get_config()->rtls_role;

    if(frame[0] == 'M') {
        if (role == ANCHOR) {
            if (frame[1] == 'R') {
                return handle_meas_req(frame, frame_length);
            }
        } else if (role == TAG) {
            if (frame[1] == 'I') {
                return handle_meas_init(frame, frame_length);
            }
        }
        else {
            LOG_WRN("%s: message type is not applicable for device", __func__);
            return -ENOTSUP;
        }
    } else {
        LOG_WRN("%s: received message is not supported", __func__);
        return -ENOTSUP;
    }
}

static void rx_ok_handler(const dwt_cb_data_t *cb_data) {
    uint16_t frame_length = cb_data->datalength;

    if (frame_length <= 0 || frame_length > FRAME_LENGTH_MAX) {
        LOG_ERR("%s: invalid frame length", __func__);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return;
    }

    uint8_t rx_buffer[FRAME_LENGTH_MAX];
    dwt_readrxdata(&rx_buffer[0], frame_length, 0);

    if (0) {
        for (int i = 0; i < frame_length; i++) {
            printk("%02x", rx_buffer[i]);
        }
        printk("\n");
    }

    int ret = handle_frame(&rx_buffer[0], frame_length);
    if (ret) {
        LOG_WRN("%s: message was not handled", __func__);
    }

    dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rx_timeout_handler(const dwt_cb_data_t *cb_data) {
    LOG_DBG("%s:", __func__);
}

static void rx_error_handler(const dwt_cb_data_t *cb_data) {
    LOG_DBG("%s:", __func__);
}

static void tx_done_handler(const dwt_cb_data_t *cb_data) {
    LOG_DBG("%s:", __func__);
}

int start_measurement(const uint32_t target_id) {
    rtls_role_t rtls_role = dev_mgmt_get_config()->rtls_role;    
    uint32_t gateway_id = dev_mgmt_get_config()->device_id;

    int ret;
    if (rtls_role == TAG) {
        LOG_ERR("%s: not supported for tags", __func__);
        return -ENOTSUP;
    } else if (rtls_role == GATEWAY_ANCHOR) {
        ret = send_measurement_request(target_id);
        if (ret) {
            LOG_ERR("%s: failed to send measurement request %d", __func__, ret);
            return ret;
        }
    }

    ret = send_measurement_init(target_id, gateway_id);
    if (ret) {
        LOG_ERR("%s: failed to send measurement init %d", __func__, ret);
        return ret;
    }

    return 0;
}

static int radio_init(void) {
    openspi();

    reset_DW1000();
    port_set_dw1000_slowrate();
    int ret = dwt_initialise(DWT_LOADUCODE | DWT_READ_OTP_PID | DWT_READ_OTP_LID);
    if (ret == DWT_ERROR) {
        LOG_ERR("%s: err - init failed %d", __func__, ret);
        return ret;
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setleds(1);
    
    return 0;
}

int radio_handler_init(void) {
    int ret = radio_init();
    if (ret) {
        LOG_ERR("%s: failed to initialize radio", __func__);
    }

    k_work_init(&isr_work, isr_handler);

    dwt_setcallbacks(tx_done_handler, rx_ok_handler, rx_timeout_handler, rx_error_handler);

    port_set_deca_isr(isr_wrapper);

    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO |
                    DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE |
                    DWT_INT_RFSL | DWT_INT_SFDT, 1);

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    uint32_t partid = dwt_getpartid();
    LOG_INF("%s: partid: %08x", __func__, partid);

    return 0;
}

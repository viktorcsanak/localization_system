#include "include/radio_handler.h"
#include "zephyr/sys/util.h"

#include <device_config.h>

#include <deca_device_api.h>
#include <deca_regs.h>
#include <deca_spi.h>
#include <port.h>

#include <string.h>
#include <sys/_stdint.h>
#include <sys/errno.h>
#include <zephyr/kernel.h>
#include <zephyr/random/random.h>
//#include <zephyr/dsp/dsp.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(radio_handler, CONFIG_LOG_RADIO_HANDLER_LEVEL);

#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define FRAME_LENGTH_MAX 127

#define DWT_TIME_UNITS (1.0/499.2e6/128.0) //!< = 15.65e-12 s
#define SPEED_OF_LIGHT 299702547
#define TWR_REPLY_DELAY_US 10000
#define UUS_TO_DWT_TIME 65536

/** @brief {'S', 'I', target_id[4], checksum[2]}*/
#define SCAN_INIT_FRAME_LENGTH 8
/** @brief {'S', 'R', target_id[4], inititator_id[4], checksum[2]}*/
#define SCAN_RESP_FRAME_LENGTH 12
/** @brief {'M', 'R', target_id[4], inititator_id[4], checksum[2]}*/
#define MEAS_REQ_FRAME_LENGTH 12
/** @brief {'M', 'P', target_id[4], initiator_id[4], pol_tx_ts[5], checksum[2]}*/
#define MEAS_POLL_FRAME_LENGTH 17
/** @brief {'M', 'A', target_id[4], initiator_id[4], pol_tx_ts[5], ans_tx_ts[5], poll_rx_ts[5], checksum[2]}*/
#define MEAS_ANS_FRAME_LENGTH 27
/** @brief {'M', 'F', target_id[4], initiator_id[4], pol_tx_ts[5], ans_tx_ts[5], poll_rx_ts[5], ans_rx_ts[5], fin_tx_ts[5], checksum[2]} */
#define MEAS_FIN_FRAME_LENGTH 37

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

K_FIFO_DEFINE(anchor_fifo);
K_FIFO_DEFINE(meas_fifo);

struct anchor_record {
    void *fifo_reserved;
    uint32_t anchor_id;
    int64_t tof_dtu;
};

struct range_work_container {
    struct k_work_delayable work;
    struct anchor_record record;
};

static int send_meas_request(const uint32_t initiator_id);

static void range_anchors(struct k_work *item) {
    struct range_work_container *container = CONTAINER_OF(k_work_delayable_from_work(item), struct range_work_container, work);

    send_meas_request(container->record.anchor_id);
    LOG_INF("%s: got anchor record with device id %08x from FIFO", __func__, container->record.anchor_id);

    k_free(container);
}

static void scan_timer_expiry(struct k_timer *timer) {
    LOG_DBG("%s:", __func__);

    if (k_fifo_is_empty(&anchor_fifo)) {
        LOG_INF("%s: FIFO is empty", __func__);
        return;
    }

    uint8_t cnt = 0;
    while (!k_fifo_is_empty(&anchor_fifo)) {
        struct range_work_container *container = k_calloc(1, sizeof(struct range_work_container));
        
        struct anchor_record *record = k_fifo_get(&anchor_fifo, K_NO_WAIT);
        memcpy(&container->record, record, sizeof(struct anchor_record));
        k_free(record);
        
        k_work_init_delayable(&container->work, range_anchors);
        int ret = k_work_schedule(&container->work, K_MSEC(cnt * 100));
        if (ret <= 0) {
            LOG_WRN("%s: failed to schedule work item", __func__);
            k_free(container);
        }

        cnt++;
    }
}

K_TIMER_DEFINE(scan_timer, scan_timer_expiry, NULL);

static void meas_timer_expiry(struct k_timer *timer) {
    LOG_INF("%s:", __func__);

    while (!k_fifo_is_empty(&meas_fifo)) {
        k_free(k_fifo_get(&meas_fifo, K_NO_WAIT));
    }
}

K_TIMER_DEFINE(meas_timer, meas_timer_expiry, NULL);

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
    uint8_t rand;
    sys_csrand_get(&rand, sizeof(uint8_t));
    return ((device_id % 127) + rand);
}

static void tx_start(struct tx_data *tx_data) {
    dwt_forcetrxoff();

    dwt_writetxdata(tx_data->data_length, &tx_data->tx_buffer[0], 0);

    dwt_writetxfctrl(tx_data->data_length, 0, 0);

    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

static void tx_immediate(struct k_work *item) {
    LOG_DBG("%s:", __func__);
    struct tx_container *container = CONTAINER_OF(item, struct tx_container, tx_work);
    struct tx_data *tx_data = &container->tx_data;

    tx_start(tx_data);

    k_free(container);
}

static void tx_schedule(struct k_work *item) {
    LOG_DBG("%s:", __func__);
    struct tx_schedule_container *container = CONTAINER_OF(k_work_delayable_from_work(item), struct tx_schedule_container, tx_work);
    struct tx_data *tx_data = &container->tx_data;

    tx_start(tx_data);

    k_free(container);
}

static int  send_scan_init(void) {
    LOG_DBG("%s:", __func__);
    struct tx_container *container = (struct tx_container *)k_calloc(1, sizeof(struct tx_container));
    if (container == NULL) {
        LOG_ERR("%s: failed to allocate container", __func__);
        return -ENOMEM;
    }

    struct tx_data *tx_data = &container->tx_data;
    tx_data->data_length = SCAN_INIT_FRAME_LENGTH;
    memset(&tx_data->tx_buffer[0], 0, SCAN_INIT_FRAME_LENGTH);

    tx_data->tx_buffer[0] = 'S';
    tx_data->tx_buffer[1] = 'I';

    uint32_t target_id = dev_mgmt_get_config()->device_id;
    memcpy(&tx_data->tx_buffer[2], &target_id, sizeof(target_id));

    k_work_init(&container->tx_work, tx_immediate);
    int ret = k_work_submit(&container->tx_work);
    if (ret < 0) {
        LOG_ERR("%s: failed to submit work item %d", __func__, ret);
        return ret;
    }

    return 0;
}

static int send_scan_resp(uint32_t target_id) {
    LOG_DBG("%s:", __func__);
    struct tx_schedule_container *container = (struct tx_schedule_container *)k_calloc(1, sizeof(struct tx_schedule_container));
    if (container == NULL) {
        LOG_ERR("%s: failed to allocate container", __func__);
        return -ENOMEM;
    }

    struct tx_data *tx_data = &container->tx_data;
    tx_data->data_length = SCAN_RESP_FRAME_LENGTH;
    memset(&tx_data->tx_buffer[0], 0, SCAN_RESP_FRAME_LENGTH);

    tx_data->tx_buffer[0] = 'S';
    tx_data->tx_buffer[1] = 'R';

    uint32_t initiator_id = dev_mgmt_get_config()->device_id;
    memcpy(&tx_data->tx_buffer[2], &target_id, sizeof(target_id));
    memcpy(&tx_data->tx_buffer[6], &initiator_id, sizeof(initiator_id));

    uint32_t delay = device_id_sleep();
    k_work_init_delayable(&container->tx_work, tx_schedule);
    int ret = k_work_schedule(&container->tx_work, K_MSEC(delay));
    if (ret < 0) {
        LOG_ERR("%s: failed to schedule work item %d", __func__, ret);
        return ret;
    }

    return 0;
}

static int send_meas_request(const uint32_t initiator_id) {
    struct tx_container *container = (struct tx_container *)k_calloc(1, sizeof(struct tx_container));
    if (container == NULL) {
        LOG_ERR("%s: failed to allocate container", __func__);
        return -ENOMEM;
    }

    LOG_DBG("%s:", __func__);

    struct tx_data *tx_data = &container->tx_data;
    tx_data->data_length = MEAS_REQ_FRAME_LENGTH;
    memset(&tx_data->tx_buffer[0], 0, MEAS_REQ_FRAME_LENGTH);

    tx_data->tx_buffer[0] = 'M';
    tx_data->tx_buffer[1] = 'R';

    uint32_t target_id = dev_mgmt_get_config()->device_id;
    memcpy(&tx_data->tx_buffer[2], &target_id, sizeof(target_id));
    memcpy(&tx_data->tx_buffer[6], &initiator_id, sizeof(initiator_id));

    k_work_init(&container->tx_work, tx_immediate);
    int ret = k_work_submit(&container->tx_work);
    if (ret < 0) {
        LOG_ERR("%s: failed to submit work item %d", __func__, ret);
        return ret;
    }

    return 0;
}

static int send_meas_poll(uint32_t target_id, uint64_t poll_tx_ts) {
    uint8_t poll_msg[MEAS_POLL_FRAME_LENGTH];
    memset(&poll_msg[0], 0, MEAS_POLL_FRAME_LENGTH);
    poll_msg[0] = 'M';
    poll_msg[1] = 'P';

    uint32_t initiator_id = dev_mgmt_get_config()->device_id;
    memcpy(&poll_msg[2], &target_id, sizeof(target_id));
    memcpy(&poll_msg[6], &initiator_id, sizeof(initiator_id));
    memcpy(&poll_msg[10], &poll_tx_ts, 5 * sizeof(uint8_t));

    dwt_writetxdata(sizeof(poll_msg), poll_msg, 0);
    dwt_writetxfctrl(sizeof(poll_msg), 0, 0);

    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (ret) {
        LOG_ERR("%s: failed to send cal_poll message", __func__);
        return ret;
    }

    return 0;
}

static void send_meas_ans(uint32_t initiator_id, uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t ans_tx_ts) {
    uint8_t ans_msg[MEAS_ANS_FRAME_LENGTH];
    memset(&ans_msg[0], 0, MEAS_ANS_FRAME_LENGTH);
    ans_msg[0] = 'M';
    ans_msg[1] = 'A';

    uint32_t target_id = dev_mgmt_get_config()->device_id;
    memcpy(&ans_msg[2], &target_id, sizeof(target_id));
    memcpy(&ans_msg[6], &initiator_id, sizeof(initiator_id));
    memcpy(&ans_msg[10], &poll_tx_ts, 5 * sizeof(uint8_t));
    memcpy(&ans_msg[15], &ans_tx_ts, 5 * sizeof(uint8_t));
    memcpy(&ans_msg[20], &poll_rx_ts, 5 * sizeof(uint8_t));

    dwt_writetxdata(sizeof(ans_msg), ans_msg, 0);
    dwt_writetxfctrl(sizeof(ans_msg), 0, 0);

    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (ret) {
        LOG_ERR("%s: failed to send cal_poll message", __func__);
        return;
    }
}

static int send_meas_fin(uint32_t target_id, uint64_t poll_tx_ts, uint64_t poll_rx_ts, uint64_t ans_tx_ts, uint64_t ans_rx_ts, uint64_t fin_tx_ts) {
    uint8_t fin_msg[MEAS_FIN_FRAME_LENGTH];
    memset(&fin_msg[0], 0, MEAS_FIN_FRAME_LENGTH);
    fin_msg[0] = 'M';
    fin_msg[1] = 'F';

    uint32_t initiator_id = dev_mgmt_get_config()->device_id;
    memcpy(&fin_msg[2], &target_id, sizeof(target_id));
    memcpy(&fin_msg[6], &initiator_id, sizeof(initiator_id));
    memcpy(&fin_msg[10], &poll_tx_ts, 5 * sizeof(uint8_t));
    memcpy(&fin_msg[15], &ans_tx_ts, 5 * sizeof(uint8_t));
    memcpy(&fin_msg[20], &poll_rx_ts, 5 * sizeof(uint8_t));
    memcpy(&fin_msg[25], &ans_rx_ts, 5 * sizeof(uint8_t));
    memcpy(&fin_msg[30], &fin_tx_ts, 5 * sizeof(uint8_t));

    dwt_writetxdata(sizeof(fin_msg), fin_msg, 0);
    dwt_writetxfctrl(sizeof(fin_msg), 0, 0);

    int ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    if (ret) {
        LOG_ERR("%s: failed to send cal_poll message", __func__);
        return -ECANCELED;
    }

    return 0;
}

static int handle_scan_init(uint8_t *frame, uint16_t frame_length) {
    if (SCAN_INIT_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, SCAN_INIT_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }

    uint32_t target_id;
    memcpy(&target_id, &frame[2], sizeof(target_id));
    
    LOG_DBG("%s: received scan_init from %08x", __func__, target_id);

    return send_scan_resp(target_id);
}

static int handle_scan_resp(uint8_t *frame, uint16_t frame_length) {
    if (SCAN_RESP_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, SCAN_RESP_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }

    uint32_t device_id = dev_mgmt_get_config()->device_id;
    uint32_t initiator_id = 0, target_id = 0;
    memcpy(&target_id, &frame[2], sizeof(target_id));
    memcpy(&initiator_id, &frame[6], sizeof(initiator_id));
    
    LOG_DBG("%s: received scan_resp from %08x", __func__, initiator_id);

    if (device_id != target_id) {
        LOG_INF("%s: message was not addressed to this device %08x vs %08x", __func__, target_id, device_id);
        return -ECANCELED;
    }

    k_timer_start(&scan_timer, K_MSEC(400), K_MSEC(0));

    struct anchor_record *anchor_record = k_calloc(1, sizeof(struct anchor_record));
    anchor_record->anchor_id = initiator_id;
    k_fifo_put(&anchor_fifo, anchor_record);

    return 0;
}

static int handle_meas_req(uint8_t *frame, uint16_t frame_length) {
    LOG_DBG("%s:", __func__);
    if (MEAS_REQ_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, MEAS_REQ_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }

    uint32_t device_id = dev_mgmt_get_config()->device_id;
    uint32_t target_id = 0, initiator_id = 0;
    memcpy(&target_id, &frame[2], sizeof(target_id));
    memcpy(&initiator_id, &frame[6], sizeof(initiator_id));

    if (device_id != initiator_id) {
        LOG_INF("%s: message was not addressed to this device %08x vs %08x", __func__, target_id, device_id);
        return -ECANCELED;
    }

    uint64_t req_rx_ts;
    dwt_readrxtimestamp((uint8_t *)&req_rx_ts);
    uint64_t poll_tx_time = (req_rx_ts + ((TWR_REPLY_DELAY_US) * UUS_TO_DWT_TIME)) >> 8;

    dwt_forcetrxoff();
    dwt_setdelayedtrxtime(poll_tx_time);

    uint64_t poll_tx_ts = (((uint64)(poll_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    return send_meas_poll(target_id, poll_tx_ts);
}

static int handle_meas_poll(uint8_t *frame, uint16_t frame_length) {
    LOG_DBG("%s:", __func__);
    if (MEAS_POLL_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, MEAS_POLL_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }

    uint32_t device_id = dev_mgmt_get_config()->device_id;

    uint32_t target_id = *(uint32_t *)(&frame[2]);
    uint32_t initiator_id = *(uint32_t *)(&frame[6]);

    uint64_t poll_tx_ts = 0;
    memcpy(&poll_tx_ts, &frame[10], 5 * sizeof(uint8_t));
    
    if (device_id != target_id) {
        LOG_INF("%s: meas_poll was not addressed to this device %08x vs %08x", __func__, device_id, target_id);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -ECANCELED;
    }

    uint64_t poll_rx_ts;
    dwt_readrxtimestamp((uint8_t *)&poll_rx_ts);
    uint64_t ans_tx_time = (poll_rx_ts + (TWR_REPLY_DELAY_US * UUS_TO_DWT_TIME)) >> 8;

    dwt_forcetrxoff();
    dwt_setdelayedtrxtime(ans_tx_time);

    uint64_t ans_tx_ts = (((uint64)(ans_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    send_meas_ans(initiator_id, poll_tx_ts, poll_rx_ts, ans_tx_ts);

    return 0;
}

static int handle_meas_ans(uint8_t *frame, uint16_t frame_length) {
    LOG_DBG("%s:", __func__);
    if (MEAS_ANS_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, MEAS_ANS_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }
    uint32_t target_id = *(uint32_t *)(&frame[2]);
    uint32_t initiator_id = *(uint32_t *)(&frame[6]);
    uint64_t poll_tx_ts = 0, ans_tx_ts = 0, poll_rx_ts = 0;
    memcpy(&poll_tx_ts, &frame[10], 5 * sizeof(uint8_t));
    memcpy(&ans_tx_ts, &frame[15], 5 * sizeof(uint8_t));
    memcpy(&poll_rx_ts, &frame[20], 5 * sizeof(uint8_t));

    uint32_t device_id = dev_mgmt_get_config()->device_id;

    if (device_id != initiator_id) {
        LOG_INF("%s: meas_ans was not addressed to this device %08x vs %08x", __func__, initiator_id, device_id);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -ECANCELED;
    }

    uint64_t ans_rx_ts;
    dwt_readrxtimestamp((uint8_t *)&ans_rx_ts);
    uint64_t fin_tx_time = (ans_rx_ts + (TWR_REPLY_DELAY_US * UUS_TO_DWT_TIME)) >> 8;

    dwt_forcetrxoff();
    dwt_setdelayedtrxtime(fin_tx_time);

    uint64_t fin_tx_ts = (((uint64)(fin_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

    send_meas_fin(target_id, poll_tx_ts, poll_rx_ts, ans_tx_ts, ans_rx_ts, fin_tx_ts);
    return 0;
}

static int handle_meas_fin(uint8_t *frame, uint16_t frame_length) {
    LOG_DBG("%s:", __func__);
    if (MEAS_FIN_FRAME_LENGTH != frame_length) {
        LOG_ERR("%s: read data length %d does not equal predefined %d", __func__, frame_length, MEAS_FIN_FRAME_LENGTH);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -EINVAL;
    }

    uint32_t target_id = *(uint32_t *)(&frame[2]);
    uint32_t initiator_id = *(uint32_t *)(&frame[6]);
    uint64_t poll_tx_ts = 0, ans_tx_ts = 0, poll_rx_ts = 0, ans_rx_ts = 0, fin_tx_ts = 0;
    memcpy(&poll_tx_ts, &frame[10], 5 * sizeof(uint8_t));
    memcpy(&ans_tx_ts, &frame[15], 5 * sizeof(uint8_t));
    memcpy(&poll_rx_ts, &frame[20], 5 * sizeof(uint8_t));
    memcpy(&ans_rx_ts, &frame[25], 5 * sizeof(uint8_t));
    memcpy(&fin_tx_ts, &frame[30], 5 * sizeof(uint8_t));

    uint32_t device_id = dev_mgmt_get_config()->device_id;
    LOG_DBG("%s:", __func__);

    if (device_id != target_id) {
        LOG_INF("%s: meas_fin was not addressed to this device %08x vs %08x", __func__, device_id, target_id);
        dwt_rxenable(DWT_START_RX_IMMEDIATE);
        return -ECANCELED;
    }

    uint64_t fin_rx_ts = 0;
    dwt_readrxtimestamp((uint8_t *)&fin_rx_ts);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    double initiator_round_time = (double)(ans_rx_ts - poll_tx_ts);
    double responder_round_time = (double)(fin_rx_ts - ans_tx_ts);
    double initiator_resp_time = (double)(fin_tx_ts - ans_rx_ts);
    double responder_resp_time = (double)(ans_tx_ts - poll_rx_ts);

    double Ra = initiator_round_time;
    double Rb = responder_round_time;
    double Da = initiator_resp_time;
    double Db = responder_resp_time;

    int64_t tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));
    double tof = tof_dtu * DWT_TIME_UNITS;
    double distance = tof * SPEED_OF_LIGHT;

    struct anchor_record *record = k_calloc(1, sizeof(struct anchor_record));
    record->anchor_id = initiator_id;
    record->tof_dtu = tof_dtu;
    k_fifo_put(&meas_fifo, record);

    k_timer_start(&meas_timer, K_MSEC(120), K_MSEC(0));

    LOG_INF("%s: measurement finished with device %08x offset is tof_dtu: %lld at distance %lf", __func__, initiator_id, tof_dtu, distance);

    return 0;
}

static int handle_frame(uint8_t *frame, uint16_t frame_length) {
    rtls_role_t role = dev_mgmt_get_config()->rtls_role;

    if (role == TAG) {
        if (frame[0] == 'S' && frame[1] == 'R') {
            return handle_scan_resp(frame, frame_length);
        } else if (frame[0] == 'M') {
            if (frame[1] == 'P') {
                return handle_meas_poll(frame, frame_length);
            } else if (frame[1] == 'F') {
                return handle_meas_fin(frame, frame_length);
            }
        }
    } else if (role == ANCHOR) {
        if (frame[0] == 'S' && frame[1] == 'I') {
            return handle_scan_init(frame, frame_length);
        } else if (frame[0] == 'M') {
            if (frame[1] == 'R') {
                return handle_meas_req(frame, frame_length);
            } else if (frame[1] == 'A') {
                return handle_meas_ans(frame, frame_length);
            }
        }
    } else {
        LOG_WRN("%s: received message is not supported", __func__);
        return -ENOTSUP;
    }

    return 0;
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
        LOG_WRN("%s: message %04x was not handled", __func__, *((uint16_t *)(&rx_buffer[0])));
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

int start_measurement(void) {
    send_scan_init();

    k_timer_start(&scan_timer, K_MSEC(400), K_MSEC(0));

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

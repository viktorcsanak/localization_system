/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Simple TX example code
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 * Copyright 2019 (c) Frederic Mes, RTLOC.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
#include "../include/deca_device_api.h"
#include "../include/deca_regs.h"
#include "../include/deca_spi.h"
#include "../include/port.h"

#include <zephyr/sys/printk.h>

/* Example application name and version to display on console. */
#define APP_NAME "SIMPLE TX v1.3"

/* Default communication configuration. */
static dwt_config_t config = {
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

/* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
#define BLINK_FRAME_SN_IDX 1

#define TX_DELAY_MS 1000

int dw_tx_main(void)
{
    printk(APP_NAME);

    reset_DW1000();
    port_set_dw1000_slowrate();

    openspi();

    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        printk("INIT FAILED");
        while (1)
        { };
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);

    dwt_setleds(1);

    while(1)
    {
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0);

        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        { };

        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        Sleep(TX_DELAY_MS);

        tx_msg[BLINK_FRAME_SN_IDX]++;
    }
}

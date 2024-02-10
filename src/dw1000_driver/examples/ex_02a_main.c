/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Simple RX example code
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
#define APP_NAME "SIMPLE RX v1.3"

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

#define FRAME_LEN_MAX 127
static uint8 rx_buffer[FRAME_LEN_MAX];

static uint32 status_reg = 0;
static uint16 frame_len = 0;

int dw_rx_main(void)
{
    printk(APP_NAME);

    openspi();

    reset_DW1000();
    port_set_dw1000_slowrate();
    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        printk("INIT FAILED");
        while (1)
        { };
    }
    port_set_dw1000_fastrate();

    dwt_configure(&config);

    dwt_setleds(1);

    while (1)
    {
        int i;

        for (i = 0 ; i < FRAME_LEN_MAX; i++ )
        {
            rx_buffer[i] = 0;
        }

        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= FRAME_LEN_MAX)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
                printk("%s\r\n", rx_buffer);
            }

            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
        }
        else
        {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}

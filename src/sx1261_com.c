/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle the sx1261 radio used for LBT/Spectral Scan.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <string.h>

#include "loragw_com.h"
#include "sx1261_com.h"
#include "loragw_aux.h"
#include "loragw_mcu.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_LBT == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return -1;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return -1;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */


/**
@brief A generic pointer to the COM device (file descriptor)
*/
static lgw_com_write_mode_t _sx1261_write_mode = LGW_COM_WRITE_MODE_SINGLE;
static uint8_t _sx1261_spi_req_nb = 0;
/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */


int sx1261_com_w(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    /* Check input parameters */
    CHECK_NULL(data);

    uint8_t command_size = size + 6; /* 5 bytes: REQ metadata, 1 byte: op_code */
    uint8_t in_out_buf[command_size];
    int a;
    int i;

    /* prepare command */
    /* Request metadata */
    in_out_buf[0] = _sx1261_spi_req_nb; /* Req ID */
    in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE; /* Req type */
    in_out_buf[2] = MCU_SPI_TARGET_SX1261; /* MCU -> SX1302 */
    in_out_buf[3] = (uint8_t)((size + 1) >> 8); /* payload size + op_code */
    in_out_buf[4] = (uint8_t)((size + 1) >> 0); /* payload size + op_code */
    /* RAW SPI frame */
    in_out_buf[5] = (uint8_t)op_code;
    for (i = 0; i < size; i++) {
        in_out_buf[i + 6] = data[i];
    }

    if (_sx1261_write_mode == LGW_COM_WRITE_MODE_BULK) {
        a = mcu_spi_store(in_out_buf, command_size);
        _sx1261_spi_req_nb += 1;
    } else {
        a = mcu_spi_write(in_out_buf, command_size);
    }

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB SX1261 WRITE FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB SX1261 write success\n");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_r(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    /* Check input parameters */
    CHECK_NULL(data);

    uint8_t command_size = size + 6; /* 5 bytes: REQ metadata, 1 byte: op_code */
    uint8_t in_out_buf[command_size];
    int a;
    int i;

    /* prepare command */
    /* Request metadata */
    in_out_buf[0] = _sx1261_spi_req_nb; /* Req ID */
    in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE; /* Req type */
    in_out_buf[2] = MCU_SPI_TARGET_SX1261; /* MCU -> SX1302 */
    in_out_buf[3] = (uint8_t)((size + 1) >> 8); /* payload size + op_code */
    in_out_buf[4] = (uint8_t)((size + 1) >> 0); /* payload size + op_code */
    /* RAW SPI frame */
    in_out_buf[5] = (uint8_t)op_code;
    for (i = 0; i < size; i++) {
        in_out_buf[i + 6] = data[i];
    }
    if (_sx1261_write_mode == LGW_COM_WRITE_MODE_BULK) {
        /* makes no sense to read in bulk mode, as we can't get the result */
        printf("ERROR: USB READ BURST FAILURE - bulk mode is enabled\n");
        return -1;
    } else {
        a = mcu_spi_write(in_out_buf, command_size);
    }

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB SX1261 WRITE FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB SX1261 write success\n");
        memcpy(data, in_out_buf + 6, size); /* remove the first bytes, keep only the payload */
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_set_write_mode(lgw_com_write_mode_t write_mode) {
    if (write_mode >= LGW_COM_WRITE_MODE_UNKNOWN) {
        printf("ERROR: %s: wrong write mode\n", __FUNCTION__);
        return -1;
    }

    DEBUG_PRINTF("INFO: setting SX1261 USB write mode to %s\n", (write_mode == LGW_COM_WRITE_MODE_SINGLE) ? "SINGLE" : "BULK");

    _sx1261_write_mode = write_mode;

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_com_flush(void) {
    int a = 0;

    /* Check input parameters */
    if (_sx1261_write_mode != LGW_COM_WRITE_MODE_BULK) {
        printf("ERROR: %s: cannot flush in single write mode\n", __FUNCTION__);
        return -1;
    }

    /* Restore single mode after flushing */
    _sx1261_write_mode = LGW_COM_WRITE_MODE_SINGLE;

    if (_sx1261_spi_req_nb == 0) {
        printf("INFO: no SX1261 SPI request to flush\n");
        return 0;
    }

    DEBUG_MSG("INFO: flushing SX1261 USB write buffer\n");
    a = mcu_spi_flush();
    if (a != 0) {
        printf("ERROR: Failed to flush sx1261 USB write buffer\n");
    }

    /* reset the pending request number */
    _sx1261_spi_req_nb = 0;

    return a;
}

/* --- EOF ------------------------------------------------------------------ */

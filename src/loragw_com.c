/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Functions to abstract the communication interface used to communicate with
    the concentrator.
    Single-byte read/write and burst read/write.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */

#include "loragw_com.h"
#include "loragw_aux.h"
#include "loragw_mcu.h"
#include "serial_port.h"

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* strncmp */



/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_COM_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_COM_ERROR;}
#endif

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */
static lgw_com_write_mode_t _lgw_write_mode = LGW_COM_WRITE_MODE_SINGLE;
static uint8_t _lgw_spi_req_nb = 0;

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */


/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_com_open(const char * com_path) {
    int x;
    s_ping_info gw_info;
    s_status mcu_status;

    if (serial_isopen() == 0) {
        DEBUG_MSG("WARNING: CONCENTRATOR WAS ALREADY CONNECTED\n");
        lgw_com_close();
    }

    x = serial_open(com_path);

    if (x != 0) {
        printf("ERROR: failed to open the port\n");
        return LGW_COM_ERROR;
    }

    /* Initialize pseudo-random generator for MCU request ID */
    srand(0);
    /* Check MCU version (ignore first char of the received version (release/debug) */
    printf("INFO: Connect to MCU\n");
    if (mcu_ping(&gw_info) != 0) {
        printf("ERROR: failed to ping the concentrator MCU\n");
        return LGW_COM_ERROR;
    }
    if (strncmp(gw_info.version + 1, mcu_version_string, sizeof mcu_version_string) != 0) {
        printf("WARNING: MCU version mismatch (expected:%s, got:%s)\n", mcu_version_string, gw_info.version);
    }
    printf("INFO: Concentrator MCU version is %s\n", gw_info.version);

    /* Get MCU status */
    if (mcu_get_status( &mcu_status) != 0) {
        printf("ERROR: failed to get status from the concentrator MCU\n");
        return LGW_COM_ERROR;
    }
    printf("INFO: MCU status: sys_time:%u temperature:%.1foC\n", mcu_status.system_time_ms, mcu_status.temperature);

    /* Reset SX1302 */
    x  = mcu_gpio_write(0, 1, 1); /*   set PA1 : POWER_EN */
    x |= mcu_gpio_write(0, 2, 1); /*   set PA2 : SX1302_RESET active */
    x |= mcu_gpio_write(0, 2, 0); /* unset PA2 : SX1302_RESET inactive */
    /* Reset SX1261 (LBT / Spectral Scan) */
    x |= mcu_gpio_write(0, 8, 0); /*   set PA8 : SX1261_NRESET active */
    x |= mcu_gpio_write(0, 8, 1); /* unset PA8 : SX1261_NRESET inactive */
    if (x != 0) {
        printf("ERROR: failed to reset SX1302\n");
        return LGW_COM_ERROR;
    }

    return LGW_COM_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* SPI release */
int lgw_com_close(void) {
    if (serial_isopen() != 0) {
        printf("ERROR: concentrator is not connected\n");
        return -1;
    }

    int x, err = LGW_COM_SUCCESS;

    /* Reset SX1302 before closing */
    x  = mcu_gpio_write(0, 1, 1); /*   set PA1 : POWER_EN */
    x |= mcu_gpio_write(0, 2, 1); /*   set PA2 : SX1302_RESET active */
    x |= mcu_gpio_write(0, 2, 0); /* unset PA2 : SX1302_RESET inactive */
    /* Reset SX1261 (LBT / Spectral Scan) */
    x |= mcu_gpio_write(0, 8, 0); /*   set PA8 : SX1261_NRESET active */
    x |= mcu_gpio_write(0, 8, 1); /* unset PA8 : SX1261_NRESET inactive */
    if (x != 0) {
        printf("ERROR: failed to reset SX1302\n");
        err = LGW_COM_ERROR;
    }

    /* close file & deallocate file descriptor */
    x = serial_close();
    if (x != 0) {
        printf("ERROR: failed to close USB file\n");
        err = LGW_COM_ERROR;
    }

    /* determine return code */
    if (err != 0) {
        printf("ERROR: USB PORT FAILED TO CLOSE\n");
        return LGW_COM_ERROR;
    } else {
        DEBUG_MSG("Note: USB port closed\n");
        return LGW_COM_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple write */
int lgw_com_w(uint8_t spi_mux_target, uint16_t address, uint8_t data) {
    return lgw_com_wb(spi_mux_target, address, &data, 1);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Simple read */
int lgw_com_r(uint8_t spi_mux_target, uint16_t address, uint8_t *data) {
    /* Check input parameters */
    CHECK_NULL(data);
    return lgw_com_rb(spi_mux_target, address, data, 1);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_rmw(uint8_t spi_mux_target, uint16_t address, uint8_t offs, uint8_t leng, uint8_t data) {

    uint8_t command_size = 6;
    uint8_t in_out_buf[command_size];
    int a = 0;


    DEBUG_PRINTF("==> RMW register @ 0x%04X, offs:%u leng:%u value:0x%02X\n", address, offs, leng, data);

    /* prepare frame to be sent */
    in_out_buf[0] = _lgw_spi_req_nb; /* Req ID */
    in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_MODIFY_WRITE; /* Req type */
    in_out_buf[2] = (uint8_t)(address >> 8); /* Register address MSB */
    in_out_buf[3] = (uint8_t)(address >> 0); /* Register address LSB */
    in_out_buf[4] = ((1 << leng) - 1) << offs; /* Register bitmask */
    in_out_buf[5] = data << offs;

    if (_lgw_write_mode == LGW_COM_WRITE_MODE_BULK) {
        a = mcu_spi_store(in_out_buf, command_size);
        _lgw_spi_req_nb += 1;
    } else {
        a = mcu_spi_write( in_out_buf, command_size);
    }

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB WRITE FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB write success\n");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) write */
int lgw_com_wb(uint8_t spi_mux_target, uint16_t address, const uint8_t *data, uint16_t size) {

    /* Check input parameters */
    CHECK_NULL(data);

    uint16_t command_size = size + 8; /* 5 bytes: REQ metadata (MCU), 3 bytes: SPI header (SX1302) */
    uint8_t in_out_buf[command_size];
    int i;
    int a = 0;

    /* prepare command */
    /* Request metadata */
    in_out_buf[0] = _lgw_spi_req_nb; /* Req ID */
    in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE; /* Req type */
    in_out_buf[2] = MCU_SPI_TARGET_SX1302; /* MCU -> SX1302 */
    in_out_buf[3] = (uint8_t)((size + 3) >> 8); /* payload size + spi_mux_target + address */
    in_out_buf[4] = (uint8_t)((size + 3) >> 0); /* payload size + spi_mux_target + address */
    /* RAW SPI frame */
    in_out_buf[5] = spi_mux_target; /* SX1302 -> RADIO_A or RADIO_B */
    in_out_buf[6] = 0x80 | ((address >> 8) & 0x7F);
    in_out_buf[7] =        ((address >> 0) & 0xFF);
    for (i = 0; i < size; i++) {
        in_out_buf[i + 8] = data[i];
    }

    if (_lgw_write_mode == LGW_COM_WRITE_MODE_BULK) {
        a = mcu_spi_store(in_out_buf, command_size);
        _lgw_spi_req_nb += 1;
    } else {
        a = mcu_spi_write(in_out_buf, command_size);
    }

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB WRITE BURST FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB write burst success\n");
        return 0;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* Burst (multiple-byte) read */
int lgw_com_rb(uint8_t spi_mux_target, uint16_t address, uint8_t *data, uint16_t size) {
    /* Check input parameters */
    CHECK_NULL(data);

    uint16_t command_size = size + 9;  /* 5 bytes: REQ metadata (MCU), 3 bytes: SPI header (SX1302), 1 byte: dummy*/
    uint8_t in_out_buf[command_size];
    int i;
    int a = 0;

    /* prepare command */
    /* Request metadata */
    in_out_buf[0] = 0; /* Req ID */
    in_out_buf[1] = MCU_SPI_REQ_TYPE_READ_WRITE; /* Req type */
    in_out_buf[2] = MCU_SPI_TARGET_SX1302; /* MCU -> SX1302 */
    in_out_buf[3] = (uint8_t)((size + 4) >> 8); /* payload size + spi_mux_target + address + dummy byte */
    in_out_buf[4] = (uint8_t)((size + 4) >> 0); /* payload size + spi_mux_target + address + dummy byte */
    /* RAW SPI frame */
    in_out_buf[5] = spi_mux_target; /* SX1302 -> RADIO_A or RADIO_B */
    in_out_buf[6] = 0x00 | ((address >> 8) & 0x7F);
    in_out_buf[7] =        ((address >> 0) & 0xFF);
    in_out_buf[8] = 0x00; /* dummy byte */
    for (i = 0; i < size; i++) {
        in_out_buf[i + 9] = data[i];
    }

    if (_lgw_write_mode == LGW_COM_WRITE_MODE_BULK) {
        /* makes no sense to read in bulk mode, as we can't get the result */
        printf("ERROR: USB READ BURST FAILURE - bulk mode is enabled\n");
        return -1;
    } else {
        a = mcu_spi_write(in_out_buf, command_size);
    }

    /* determine return code */
    if (a != 0) {
        DEBUG_MSG("ERROR: USB READ BURST FAILURE\n");
        return -1;
    } else {
        DEBUG_MSG("Note: USB read burst success\n");
        memcpy(data, in_out_buf + 9, size); /* remove the first bytes, keep only the payload */
        return 0;
    }


}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_set_write_mode(lgw_com_write_mode_t write_mode) {
    if (write_mode >= LGW_COM_WRITE_MODE_UNKNOWN) {
        printf("ERROR: wrong write mode\n");
        return -1;
    }

    DEBUG_PRINTF("INFO: setting USB write mode to %s\n", (write_mode == LGW_COM_WRITE_MODE_SINGLE) ? "SINGLE" : "BULK");

    _lgw_write_mode = write_mode;

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_flush(void) {
    int a = 0;
    if (_lgw_write_mode != LGW_COM_WRITE_MODE_BULK) {
        printf("ERROR: %s: cannot flush in single write mode\n", __FUNCTION__);
        return -1;
    }

    /* Restore single mode after flushing */
    _lgw_write_mode = LGW_COM_WRITE_MODE_SINGLE;

    if (_lgw_spi_req_nb == 0) {
        printf("INFO: no SPI request to flush\n");
        return 0;
    }


    DEBUG_MSG("INFO: flushing USB write buffer\n");
    a = mcu_spi_flush();
    if (a != 0) {
        printf("ERROR: Failed to flush USB write buffer\n");
    }

    /* reset the pending request number */
    _lgw_spi_req_nb = 0;

    return a;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint16_t lgw_com_chunk_size(void) {
    return (uint16_t)LGW_USB_BURST_CHUNK;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_com_get_temperature(float * temperature) {
    /* Check input parameters */
    CHECK_NULL(temperature);
    s_status mcu_status;

    if (mcu_get_status(&mcu_status) != 0) {
        printf("ERROR: failed to get status from the concentrator MCU\n");
        return -1;
    }
    DEBUG_PRINTF("INFO: temperature:%.1foC\n", mcu_status.temperature);

    *temperature = mcu_status.temperature;

    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Functions used to handle LoRa concentrator SX1261 radio used to handle LBT
    and Spectral Scan.

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* strncmp */

#include "loragw_sx1261.h"
#include "loragw_com.h"
#include "loragw_aux.h"
#include "loragw_reg.h"
#include "loragw_hal.h"

#include "sx1261_com.h"

#include "sx1261_pram.var"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_LBT == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_REG_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return LGW_REG_ERROR;}
#endif

#define CHECK_ERR(a)                    if(a==-1){return LGW_REG_ERROR;}

#define DEBUG_SX1261_GET_STATUS 0

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define SX1261_PRAM_VERSION_FULL_SIZE 16 /* 15 bytes + terminating char */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

int sx1261_pram_get_version(char * version_str) {
    uint8_t buff[3 + SX1261_PRAM_VERSION_FULL_SIZE] = { 0 };
    int x;

    /* Check input parameter */
    CHECK_NULL(version_str);

    /* Get version string (15 bytes) at address 0x320 */
    buff[0] = 0x03;
    buff[1] = 0x20;
    buff[2] = 0x00; /* status */
    x = sx1261_reg_r(SX1261_READ_REGISTER, buff, 18);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: failed to read SX1261 PRAM version\n");
        return x;
    }

    /* Return full PRAM version string */
    buff[18] = '\0';
    strncpy(version_str, (char*)(buff + 3), 16); /* 15 bytes + terminating char */
    version_str[16] = '\0';

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_get_status(uint8_t * status) {
    uint8_t buff[1];

    buff[0] = 0x00;
    sx1261_reg_r(SX1261_GET_STATUS, buff, 1);

    *status = buff[0] & 0x7E; /* ignore bit 0 & 7 */

    DEBUG_PRINTF("SX1261: %s: get_status: 0x%02X (0x%02X)\n", __FUNCTION__, *status, buff[0]);

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_check_status(uint8_t expected_status) {
    int err;
    uint8_t status;

    err = sx1261_get_status(&status);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: %s: failed to get status\n", __FUNCTION__);
        return LGW_REG_ERROR;
    }

    if (status != expected_status) {
        printf("ERROR: %s: SX1261 status is not as expected: got:0x%02X expected:0x%02X\n", __FUNCTION__, status, expected_status);
        return LGW_REG_ERROR;
    }

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char * get_scan_status_str(const lgw_spectral_scan_status_t status) {
    switch (status) {
        case LGW_SPECTRAL_SCAN_STATUS_NONE:
            return "LGW_SPECTRAL_SCAN_STATUS_NONE";
        case LGW_SPECTRAL_SCAN_STATUS_ON_GOING:
            return "LGW_SPECTRAL_SCAN_STATUS_ON_GOING";
        case LGW_SPECTRAL_SCAN_STATUS_ABORTED:
            return "LGW_SPECTRAL_SCAN_STATUS_ABORTED";
        case LGW_SPECTRAL_SCAN_STATUS_COMPLETED:
            return "LGW_SPECTRAL_SCAN_STATUS_COMPLETED";
        default:
            return "LGW_SPECTRAL_SCAN_STATUS_UNKNOWN";
    }
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int sx1261_reg_w(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int com_stat;

    /* checking input parameters */
    CHECK_NULL(data);

    com_stat = sx1261_com_w(op_code, data, size);
    if (com_stat != LGW_COM_SUCCESS) {
        printf("ERROR: COM ERROR DURING SX1261 RADIO REGISTER WRITE\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_reg_r(sx1261_op_code_t op_code, uint8_t *data, uint16_t size) {
    int com_stat;

    /* checking input parameters */
    CHECK_NULL(data);

    com_stat = sx1261_com_r(op_code, data, size);
    if (com_stat != LGW_COM_SUCCESS) {
        printf("ERROR: COM ERROR DURING SX1261 RADIO REGISTER READ\n");
        return LGW_REG_ERROR;
    } else {
        return LGW_REG_SUCCESS;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_load_pram(void) {
    int i, err;
    uint8_t buff[32];
    char pram_version[SX1261_PRAM_VERSION_FULL_SIZE];
    uint32_t val, addr;

    /* Set Radio in Standby mode */
    buff[0] = (uint8_t)SX1261_STDBY_RC;
    sx1261_reg_w(SX1261_SET_STANDBY, buff, 1);

    /* Check status */
    err = sx1261_check_status(SX1261_STATUS_MODE_STBY_RC | SX1261_STATUS_READY);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: %s: SX1261 status error\n", __FUNCTION__);
        return -1;
    }

    err = sx1261_pram_get_version(pram_version);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: %s: SX1261 failed to get pram version\n", __FUNCTION__);
        return -1;
    }
    printf("SX1261: PRAM version: %s\n", pram_version);

    /* Enable patch update */
    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x10;
    err = sx1261_reg_w( SX1261_WRITE_REGISTER, buff, 3);
    CHECK_ERR(err);

    /* Load patch */
    for (i = 0; i < (int)PRAM_COUNT; i++) {
        val = pram[i];
        addr = 0x8000 + 4*i;

        buff[0] = (addr >> 8) & 0xFF;
        buff[1] = (addr >> 0) & 0xFF;
        buff[2] = (val >> 24) & 0xFF;
        buff[3] = (val >> 16) & 0xFF;
        buff[4] = (val >> 8)  & 0xFF;
        buff[5] = (val >> 0)  & 0xFF;
        err = sx1261_reg_w(SX1261_WRITE_REGISTER, buff, 6);
        CHECK_ERR(err);
    }

    /* Disable patch update */
    buff[0] = 0x06;
    buff[1] = 0x10;
    buff[2] = 0x00;
    err = sx1261_reg_w( SX1261_WRITE_REGISTER, buff, 3);
    CHECK_ERR(err);

    /* Update pram */
    buff[0] = 0;
    err = sx1261_reg_w(0xd9, buff, 0);
    CHECK_ERR(err);

    err = sx1261_pram_get_version(pram_version);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: %s: SX1261 failed to get pram version\n", __FUNCTION__);
        return -1;
    }
    printf("SX1261: PRAM version: %s\n", pram_version);

    /* Check PRAM version (only last 4 bytes) */
    if (strncmp(pram_version + 11, sx1261_pram_version_string, 4) != 0) {
        printf("ERROR: SX1261 PRAM version mismatch (got:%s expected:%s)\n", pram_version + 11, sx1261_pram_version_string);
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_calibrate(uint32_t freq_hz) {
    int err = LGW_REG_SUCCESS;
    uint8_t buff[16];

    buff[0] = 0x00;
    err = sx1261_reg_r(SX1261_GET_STATUS, buff, 1);
    CHECK_ERR(err);

    /* Run calibration */
    if ((freq_hz > 430E6) && (freq_hz < 440E6)) {
        buff[0] = 0x6B;
        buff[1] = 0x6F;
    } else if ((freq_hz > 470E6) && (freq_hz < 510E6)) {
        buff[0] = 0x75;
        buff[1] = 0x81;
    } else if ((freq_hz > 779E6) && (freq_hz < 787E6)) {
        buff[0] = 0xC1;
        buff[1] = 0xC5;
    } else if ((freq_hz > 863E6) && (freq_hz < 870E6)) {
        buff[0] = 0xD7;
        buff[1] = 0xDB;
    } else if ((freq_hz > 902E6) && (freq_hz < 928E6)) {
        buff[0] = 0xE1;
        buff[1] = 0xE9;
    } else {
        printf("ERROR: failed to calibrate sx1261 radio, frequency range not supported (%u)\n", freq_hz);
        return LGW_REG_ERROR;
    }
    err = sx1261_reg_w(SX1261_CALIBRATE_IMAGE, buff, 2);
    CHECK_ERR(err);

    /* Wait for calibration to complete */
    wait_ms(10);

    buff[0] = 0x00;
    buff[1] = 0x00;
    buff[2] = 0x00;
    err = sx1261_reg_r(SX1261_GET_DEVICE_ERRORS, buff, 3);
    CHECK_ERR(err);
    if (TAKE_N_BITS_FROM(buff[2], 4, 1) != 0) {
        printf("ERROR: sx1261 Image Calibration Error\n");
        return LGW_REG_ERROR;
    }

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_setup(void) {
    int err;
    uint8_t buff[32];

    /* Set Radio in Standby mode */
    buff[0] = (uint8_t)SX1261_STDBY_RC;
    err = sx1261_reg_w(SX1261_SET_STANDBY, buff, 1);
    CHECK_ERR(err);

    /* Check radio status */
    err = sx1261_check_status(SX1261_STATUS_MODE_STBY_RC | SX1261_STATUS_READY);
    CHECK_ERR(err);

    /* Set Buffer Base address */
    buff[0] = 0x80;
    buff[1] = 0x80;
    err = sx1261_reg_w(SX1261_SET_BUFFER_BASE_ADDRESS, buff, 2);
    CHECK_ERR(err);

    /* sensi adjust */
    buff[0] = 0x08;
    buff[1] = 0xAC;
    buff[2] = 0xCB;
    err = sx1261_reg_w(SX1261_WRITE_REGISTER, buff, 3);
    CHECK_ERR(err);

    DEBUG_MSG("SX1261: setup done\n");

    return LGW_REG_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int sx1261_set_rx_params(uint32_t freq_hz, uint8_t bandwidth) {
    int err;
    uint8_t buff[16];
    int32_t freq_reg;
    uint8_t fsk_bw_reg;

    /* Set SPI write bulk mode to optimize speed on USB */
    err = sx1261_com_set_write_mode(LGW_COM_WRITE_MODE_BULK);
    CHECK_ERR(err);

    /* Set FS */
    err = sx1261_reg_w(SX1261_SET_FS, buff, 0);
    CHECK_ERR(err);

#if DEBUG_SX1261_GET_STATUS /* need to disable spi bulk mode if enable this check */
    /* Check radio status */
    err = sx1261_check_status(SX1261_STATUS_MODE_FS | SX1261_STATUS_READY);
    CHECK_ERR(err);
#endif

    /* Set frequency */
    freq_reg = SX1261_FREQ_TO_REG(freq_hz);
    buff[0] = (uint8_t)(freq_reg >> 24);
    buff[1] = (uint8_t)(freq_reg >> 16);
    buff[2] = (uint8_t)(freq_reg >> 8);
    buff[3] = (uint8_t)(freq_reg >> 0);
    err = sx1261_reg_w(SX1261_SET_RF_FREQUENCY, buff, 4);
    CHECK_ERR(err);

    /* Configure RSSI averaging window */
    buff[0] = 0x08;
    buff[1] = 0x9B;
    buff[2] = 0x05 << 2;
    err = sx1261_reg_w(SX1261_WRITE_REGISTER, buff, 3);
    CHECK_ERR(err);

    /* Set PacketType */
    buff[0] = 0x00; /* FSK */
    err = sx1261_reg_w(SX1261_SET_PACKET_TYPE, buff, 1);
    CHECK_ERR(err);

    /* Set GFSK bandwidth */
    switch (bandwidth) {
        case BW_125KHZ:
            fsk_bw_reg = 0x0A; /* RX_BW_234300 Hz */
            break;
        case BW_250KHZ:
            fsk_bw_reg = 0x09; /* RX_BW_467000 Hz */
            break;
        default:
            printf("ERROR: %s: Cannot configure sx1261 for bandwidth %u\n", __FUNCTION__, bandwidth);
            return LGW_REG_ERROR;
    }

    /* Set modulation params for FSK */
    buff[0] = 0;    // BR
    buff[1] = 0x14; // BR
    buff[2] = 0x00; // BR
    buff[3] = 0x00; // Gaussian BT disabled
    buff[4] = fsk_bw_reg;
    buff[5] = 0x02; // FDEV
    buff[6] = 0xE9; // FDEV
    buff[7] = 0x0F; // FDEV
    err = sx1261_reg_w(SX1261_SET_MODULATION_PARAMS, buff, 8);
    CHECK_ERR(err);

    /* Set packet params for FSK */
    buff[0] = 0x00; /* Preamble length MSB */
    buff[1] = 0x20; /* Preamble length LSB 32 bits*/
    buff[2] = 0x05; /* Preamble detector lenght 16 bits */
    buff[3] = 0x20; /* SyncWordLength 32 bits*/
    buff[4] = 0x00; /* AddrComp disabled */
    buff[5] = 0x01; /* PacketType variable size */
    buff[6] = 0xff; /* PayloadLength 255 bytes */
    buff[7] = 0x00; /* CRCType 1 Byte */
    buff[8] = 0x00; /* Whitening disabled*/
    err = sx1261_reg_w(SX1261_SET_PACKET_PARAMS, buff, 9);
    CHECK_ERR(err);

    /* Set Radio in Rx continuous mode */
    buff[0] = 0xFF;
    buff[1] = 0xFF;
    buff[2] = 0xFF;
    err = sx1261_reg_w(SX1261_SET_RX, buff, 3);
    CHECK_ERR(err);

    /* Flush write (USB BULK mode) */
    err = sx1261_com_flush();
    if (err != 0) {
        printf("ERROR: %s: Failed to flush sx1261 SPI\n", __FUNCTION__);
        return -1;
    }

    /* Setting back to SINGLE BULK write mode */
    err = sx1261_com_set_write_mode(LGW_COM_WRITE_MODE_SINGLE);
    CHECK_ERR(err);

#if DEBUG_SX1261_GET_STATUS
    /* Check radio status */
    err = sx1261_check_status(SX1261_STATUS_MODE_RX | SX1261_STATUS_READY);
    CHECK_ERR(err);
#endif

    DEBUG_PRINTF("SX1261: RX params set to %u Hz (bw:0x%02X)\n", freq_hz, bandwidth);

    return LGW_REG_SUCCESS;
}

/* --- EOF ------------------------------------------------------------------ */

/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Set the sx1261 radio of the Corecell in RX continuous mode, and measure RSSI

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "loragw_aux.h"
#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_com.h"
#include "loragw_sx1261.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define BUFF_SIZE           16
#define DEFAULT_FREQ_HZ     868500000U

/* -------------------------------------------------------------------------- */
/* --- GLOBAL VARIABLES ----------------------------------------------------- */


/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DECLARATION --------------------------------------------- */

static void exit_failure();

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(void)
{
    int x;
    double arg_d = 0.0;
    unsigned int arg_u;

    uint8_t buff[BUFF_SIZE];
    uint32_t freq_hz = 868100000;
    float rssi_inst;
    uint32_t fa = DEFAULT_FREQ_HZ;
    uint32_t fb = DEFAULT_FREQ_HZ;
    uint8_t clocksource = 0;

    /* COM interfaces */
    const char com_path_default[] = "COM7";
    const char * com_path = com_path_default;
    const char sx1261_path_default[] = "COM7";
    const char * sx1261_path = sx1261_path_default;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    strncpy(boardconf.com_path, com_path, sizeof boardconf.com_path);
    boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return -1;
    }

    /* set configuration for RF chains */
    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true; /* must be enabled to proper RF matching */
    rfconf.freq_hz = fa;
    rfconf.rssi_offset = 0.0;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return -1;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true; /* must be enabled to proper RF matching */
    rfconf.freq_hz = fb;
    rfconf.rssi_offset = 0.0;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return -1;
    }

    /* Connect to the concentrator board */
    x = lgw_start();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to connect to the concentrator using COM %s\n", com_path);
        return -1;
    }

    x = sx1261_calibrate(freq_hz);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to calibrate the sx1261\n");
        exit_failure();
    }

    x = sx1261_setup();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to setup the sx1261\n");
        exit_failure();
    }

    x = sx1261_set_rx_params(freq_hz, BW_125KHZ);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to set RX params\n");
        exit_failure();
    }

    /* databuffer R/W stress test */
    for (int i = 0; i < 10; i++) {
        buff[0] = 0x00;
        buff[1] = 0x00;
        sx1261_reg_r(SX1261_GET_RSSI_INST, buff, 2);

        rssi_inst = -((float)buff[1] / 2);

        printf("\rSX1261 RSSI at %uHz: %f dBm", freq_hz, rssi_inst);
        wait_ms(100);
    }
    printf("\n");

    /* Disconnect from the concentrator board */
    x = lgw_stop();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to disconnect from the concentrator\n");
    }

    printf("Disconnected\n");
    return 0;
}

/* -------------------------------------------------------------------------- */
/* --- SUBFUNCTIONS DEFINITION ---------------------------------------------- */

static void exit_failure() {
    lgw_disconnect();
    printf("End of test for loragw_spi_sx1261.c\n");
}

/* --- EOF ------------------------------------------------------------------ */

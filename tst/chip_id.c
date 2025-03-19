/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Utility to get SX1302 chip EUI

License: Revised BSD License, see LICENSE.TXT file include in the project
*/

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_CLK_SRC     0
#define DEFAULT_FREQ_HZ     868500000U

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(void)
{
    int i, x;
    unsigned int arg_u;
    uint8_t clocksource = 0;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    uint64_t eui;

    /* SPI interfaces */
    const char com_path_default[] = "COM7";
    const char * com_path = com_path_default;

    /* Configure the gateway */
    memset(&boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = clocksource;
    strncpy(boardconf.com_path, com_path, sizeof boardconf.com_path);
    boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return -1;
    }

    memset(&rfconf, 0, sizeof rfconf);
    rfconf.enable = true; /* rf chain 0 needs to be enabled for calibration to work on sx1257 */
    rfconf.freq_hz = 868500000; /* dummy */
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return -1;
    }

    memset(&rfconf, 0, sizeof rfconf);
    rfconf.enable = (clocksource == 1) ? true : false;
    rfconf.freq_hz = 868500000; /* dummy */
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return -1;
    }

    x = lgw_start();
    if (x != 0) {
        printf("ERROR: failed to start the gateway\n");
        return -1;
    }

    /* get the concentrator EUI */
    x = lgw_get_eui(&eui);
    if (x != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to get concentrator EUI\n");
    } else {
        printf("\nINFO: concentrator EUI: 0x%016" PRIx64 "\n\n", eui);
    }

    /* Stop the gateway */
    x = lgw_stop();
    if (x != 0) {
        printf("ERROR: failed to stop the gateway\n");
        return -1;
    }
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

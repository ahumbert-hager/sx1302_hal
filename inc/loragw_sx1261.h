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


#ifndef _LORAGW_SX1261_H
#define _LORAGW_SX1261_H

/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>     /* C99 types*/
#include <stdbool.h>    /* bool type */

#include "loragw_hal.h"
#include "sx1261_defs.h"

#include "config.h"     /* library configuration options */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC MACROS -------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC CONSTANTS ----------------------------------------------------- */

static const char sx1261_pram_version_string[] = "2D06";

/* -------------------------------------------------------------------------- */
/* --- PUBLIC TYPES --------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS PROTOTYPES ------------------------------------------ */

int sx1261_reg_w(sx1261_op_code_t op_code, uint8_t *data, uint16_t size);
int sx1261_reg_r(sx1261_op_code_t op_code, uint8_t *data, uint16_t size);

int sx1261_load_pram(void);
int sx1261_calibrate(uint32_t freq_hz);
int sx1261_setup(void);
int sx1261_set_rx_params(uint32_t freq_hz, uint8_t bandwidth);

#endif

/* --- EOF ------------------------------------------------------------------ */

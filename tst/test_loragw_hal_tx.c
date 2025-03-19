/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for HAL TX capability

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

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
    uint32_t ft = DEFAULT_FREQ_HZ;
    int8_t rf_power = 0;
    uint8_t sf = 0;
    uint16_t bw_khz = 0;
    uint32_t nb_pkt = 1;
    unsigned int nb_loop = 1, cnt_loop;
    uint8_t size = 0;
    char mod[64] = "LORA";
    float br_kbps = 50;
    uint8_t fdev_khz = 25;
    int8_t freq_offset = 0;
    double arg_d = 0.0;
    unsigned int arg_u;
    int arg_i;
    char arg_s[64];
    float xf = 0.0;
    uint8_t clocksource = 0;
    uint8_t rf_chain = 0;
    uint16_t preamble = 8;
    bool invert_pol = false;
    bool no_header = false;
    bool single_input_mode = false;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_pkt_tx_s pkt;
    uint8_t tx_status;
    uint32_t count_us;
    uint32_t trig_delay_us = 1000000;
    bool trig_delay = false;

    /* SPI interfaces */
    const char com_path_default[] = "COM7";
    const char * com_path = com_path_default;


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

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true; /* rf chain 0 needs to be enabled for calibration to work on sx1257 */
    rfconf.freq_hz = ft;
    rfconf.tx_enable = true;
    rfconf.single_input_mode = single_input_mode;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return -1;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = (((rf_chain == 1) || (clocksource == 1)) ? true : false);
    rfconf.freq_hz = ft;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = single_input_mode;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return -1;
    }

    for (cnt_loop = 0; cnt_loop < nb_loop; cnt_loop++) {
        /* connect, configure and start the LoRa concentrator */
        x = lgw_start();
        if (x != 0) {
            printf("ERROR: failed to start the gateway\n");
            return -1;
        }

        /* Send packets */
        memset(&pkt, 0, sizeof pkt);
        pkt.rf_chain = rf_chain;
        pkt.freq_hz = ft;
        pkt.rf_power = rf_power;
        if (trig_delay == false) {
            pkt.tx_mode = IMMEDIATE;
        } else {
            if (trig_delay_us == 0) {
                pkt.tx_mode = ON_GPS;
            } else {
                pkt.tx_mode = TIMESTAMPED;
            }
        }
        if ( strcmp( mod, "CW" ) == 0 ) {
            pkt.modulation = MOD_CW;
            pkt.freq_offset = freq_offset;
            pkt.f_dev = fdev_khz;
        }
        else if( strcmp( mod, "FSK" ) == 0 ) {
            pkt.modulation = MOD_FSK;
            pkt.no_crc = false;
            pkt.datarate = br_kbps * 1e3;
            pkt.f_dev = fdev_khz;
        } else {
            pkt.modulation = MOD_LORA;
            pkt.coderate = CR_LORA_4_5;
            pkt.no_crc = true;
        }
        pkt.invert_pol = invert_pol;
        pkt.preamble = preamble;
        pkt.no_header = no_header;
        pkt.payload[0] = 0x40; /* Confirmed Data Up */
        pkt.payload[1] = 0xAB;
        pkt.payload[2] = 0xAB;
        pkt.payload[3] = 0xAB;
        pkt.payload[4] = 0xAB;
        pkt.payload[5] = 0x00; /* FCTrl */
        pkt.payload[6] = 0; /* FCnt */
        pkt.payload[7] = 0; /* FCnt */
        pkt.payload[8] = 0x02; /* FPort */
        for (i = 9; i < 255; i++) {
            pkt.payload[i] = i;
        }

        for (i = 0; i < (int)nb_pkt; i++) {
            if (trig_delay == true) {
                if (trig_delay_us > 0) {
                    lgw_get_instcnt(&count_us);
                    printf("count_us:%u\n", count_us);
                    pkt.count_us = count_us + trig_delay_us;
                    printf("programming TX for %u\n", pkt.count_us);
                } else {
                    printf("programming TX for next PPS (GPS)\n");
                }
            }

            if(pkt.modulation == MOD_LORA){
                pkt.datarate = sf;
            }

            switch (bw_khz) {
                case 125:
                    pkt.bandwidth = BW_125KHZ;
                    break;
                case 250:
                    pkt.bandwidth = BW_250KHZ;
                    break;
                case 500:
                    pkt.bandwidth = BW_500KHZ;
                    break;
                default:
                    pkt.bandwidth = (uint8_t)RAND_RANGE(BW_125KHZ, BW_500KHZ);
                    break;
            }

            pkt.size = size;
            pkt.payload[6] = (uint8_t)(i >> 0); /* FCnt */
            pkt.payload[7] = (uint8_t)(i >> 8); /* FCnt */

            x = lgw_send(&pkt);
            if (x != 0) {
                printf("ERROR: failed to send packet\n");
                break;
            }
            /* wait for packet to finish sending */
            do {
                wait_ms(5);
                lgw_status(pkt.rf_chain, TX_STATUS, &tx_status); /* get TX status */
            } while (tx_status != TX_FREE);

            printf("TX done\n");
        }

        printf( "\nNb packets sent: %u (%u)\n", i, cnt_loop + 1 );

        /* Stop the gateway */
        x = lgw_stop();
        if (x != 0) {
            printf("ERROR: failed to stop the gateway\n");
        }
    }

    printf("=========== Test End ===========\n");
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

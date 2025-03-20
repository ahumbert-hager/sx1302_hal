/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program for HAL RX capability

License: Revised BSD License, see LICENSE.TXT file include in the project
*/


/* -------------------------------------------------------------------------- */
/* --- DEPENDANCIES --------------------------------------------------------- */

/* fix an issue between POSIX and C99 */
#if __STDC_VERSION__ >= 199901L
    #define _XOPEN_SOURCE 600
#else
    #define _XOPEN_SOURCE 500
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include <signal.h>
#include <math.h>
#include <getopt.h>

#include "loragw_hal.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_sx1261.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define RAND_RANGE(min, max) (rand() % (max + 1 - min) + min)

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define DEFAULT_FREQ_HZ     868500000U


/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(void)
{
    /* SPI interfaces */
    const char com_path[] = "COM7";

    int i, j, x;
    uint8_t max_rx_pkt = 16;

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;

 
    unsigned long nb_pkt_crc_ok = 0, nb_loop = 0;
    int nb_pkt;

    const int32_t channel_if_mode0[9] = {
        -400000,
        -200000,
        0,
        -400000,
        -200000,
        0,
        200000,
        400000,
        -200000 /* lora service */
    };

    const uint8_t channel_rfchain_mode0[9] = { 1, 1, 1, 0, 0, 0, 0, 0, 1 };

    /* Configure the gateway */
    memset( &boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = 0;
    strncpy(boardconf.com_path, com_path, sizeof boardconf.com_path);
    boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return -1;
    }

    /* set configuration for RF chains */
    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 867500000;
    rfconf.rssi_offset = -215.4;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    rfconf.rssi_tcomp.coeff_a = 0;
    rfconf.rssi_tcomp.coeff_b = 0;
    rfconf.rssi_tcomp.coeff_c = 20.41;
    rfconf.rssi_tcomp.coeff_d = 2162.56;
    rfconf.rssi_tcomp.coeff_e = 0;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return -1;
    }

    memset( &rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 868500000;
    rfconf.rssi_offset = -215.4;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    rfconf.rssi_tcomp.coeff_a = 0;
    rfconf.rssi_tcomp.coeff_b = 0;
    rfconf.rssi_tcomp.coeff_c = 20.41;
    rfconf.rssi_tcomp.coeff_d = 2162.56;
    rfconf.rssi_tcomp.coeff_e = 0;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return -1;
    }

    /* set configuration for LoRa multi-SF channels (bandwidth cannot be set) */
    memset(&ifconf, 0, sizeof(ifconf));
    for (i = 0; i < 8; i++) {
        ifconf.enable = true;
        ifconf.rf_chain = channel_rfchain_mode0[i];
        ifconf.freq_hz = channel_if_mode0[i];
        ifconf.datarate = DR_LORA_SF7;
        if (lgw_rxif_setconf(i, &ifconf) != LGW_HAL_SUCCESS) {
            printf("ERROR: failed to configure rxif %d\n", i);
            return -1;
        }
    }

    /* set configuration for LoRa Service channel */
    memset(&ifconf, 0, sizeof(ifconf));
    ifconf.rf_chain = channel_rfchain_mode0[i];
    ifconf.freq_hz = channel_if_mode0[i];
    ifconf.datarate = DR_LORA_SF7;
    ifconf.bandwidth = BW_250KHZ;
    ifconf.implicit_crc_en = false;
    ifconf.implicit_coderate = 1;
    ifconf.implicit_hdr = false;
    ifconf.implicit_payload_length = 17;
    if (lgw_rxif_setconf(8, &ifconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxif for LoRa service channel\n");
        return -1;
    }

    /* set configuration for FSK channel */
    memset(&ifconf, 0, sizeof(ifconf));
    ifconf.rf_chain = 1;
    ifconf.freq_hz = 300000;
    ifconf.datarate = 50000;
    if (lgw_rxif_setconf(9, &ifconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: invalid configuration for FSK channel\n");
        return -1;
    }

    /* connect, configure and start the LoRa concentrator */
    x = lgw_start();
    if (x != 0) {
        printf("ERROR: failed to start the gateway\n");
        return -1;
    }

    /* get the concentrator EUI */
    uint64_t eui;
    x = lgw_get_eui(&eui);
    if (x != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to get concentrator EUI\n");
    } else {
        printf("\nINFO: concentrator EUI: 0x%016" PRIx64 "\n\n", eui);
    }

    /* configure the sx1261 */
    x = sx1261_calibrate(868100000);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to calibrate the sx1261\n");
    }

    x = sx1261_setup();
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to setup the sx1261\n");
    }

    x = sx1261_set_rx_params(868100000, BW_125KHZ);
    if (x != LGW_REG_SUCCESS) {
        printf("ERROR: Failed to set RX params\n");
    }

    
    uint8_t buff[2];
    float rssi_inst;
    /* databuffer R/W stress test */
    sx1261_reg_r(SX1261_GET_RSSI_INST, buff, 2);

    rssi_inst = -((float)buff[1] / 2);

    printf("\rSX1261 RSSI at %uHz: %f dBm\n", 868100000, rssi_inst);

    /* Loop until we have enough packets with CRC OK */
    printf("Waiting for packets...\n");


    /* set the buffer size to hold received packets */
    struct lgw_pkt_rx_s rxpkt[max_rx_pkt];
    nb_pkt_crc_ok = 0;

    while ((nb_pkt_crc_ok < 5) && (nb_loop < 500)) {
        /* fetch N packets */
        nb_pkt = lgw_receive(ARRAY_SIZE(rxpkt), rxpkt);

        if (nb_pkt == 0) {
            wait_ms(100);
            nb_loop++;
        } else {
            for (i = 0; i < nb_pkt; i++) {
                if (rxpkt[i].status == STAT_CRC_OK) {
                    nb_pkt_crc_ok += 1;
                }
                printf("\n----- %s packet -----\n", (rxpkt[i].modulation == MOD_LORA) ? "LoRa" : "FSK");
                printf("  count_us: %u\n", rxpkt[i].count_us);
                printf("  size:     %u\n", rxpkt[i].size);
                printf("  chan:     %u\n", rxpkt[i].if_chain);
                printf("  status:   0x%02X\n", rxpkt[i].status);
                printf("  datr:     %u\n", rxpkt[i].datarate);
                printf("  codr:     %u\n", rxpkt[i].coderate);
                printf("  rf_chain  %u\n", rxpkt[i].rf_chain);
                printf("  freq_hz   %u\n", rxpkt[i].freq_hz);
                printf("  snr_avg:  %.1f\n", rxpkt[i].snr);
                printf("  rssi_chan:%.1f\n", rxpkt[i].rssic);
                printf("  rssi_sig :%.1f\n", rxpkt[i].rssis);
                printf("  crc:      0x%04X\n", rxpkt[i].crc);
                for (j = 0; j < rxpkt[i].size; j++) {
                    printf("%02X ", rxpkt[i].payload[j]);
                }
                printf("\n");
            }
            printf("Received %d packets (total:%lu)\n", nb_pkt, nb_pkt_crc_ok);
        }
    }

    printf( "Nb valid packets received: %lu CRC OK\n", nb_pkt_crc_ok );


    struct lgw_pkt_tx_s pkt;
    uint8_t tx_status;
    uint32_t count_us;
    
    /* Send packets */
     memset(&pkt, 0, sizeof pkt);
     pkt.rf_chain = 0;
     pkt.freq_hz = 868500000;
     pkt.rf_power = 0;
     pkt.tx_mode = IMMEDIATE; //or
     pkt.tx_mode = TIMESTAMPED;
     
     #ifdef CW
    pkt.modulation = MOD_CW;
    pkt.freq_offset = 0;
    pkt.f_dev = 25;
    #endif
     
    #ifdef FSK
         pkt.modulation = MOD_FSK;
         pkt.no_crc = false;
         pkt.datarate = 50000;
         pkt.f_dev = 25;
    #endif
    pkt.modulation = MOD_LORA;
    pkt.coderate = CR_LORA_4_5;
    pkt.no_crc = true;
    pkt.datarate = 10;


     pkt.bandwidth = BW_250KHZ;
     pkt.size = 10;
     pkt.invert_pol = false;
     pkt.preamble = 8;
     pkt.no_header = false;
     for (int i = 0; i < 8; i++) {
         pkt.payload[i] = i;
     }
        lgw_get_instcnt(&count_us);
     pkt.count_us = count_us + 1000000;
  
     x = lgw_send(&pkt);
     if (x != 0) {
         printf("ERROR: failed to send packet\n");
     }
     /* wait for packet to finish sending */
     do {
         wait_ms(5);
         lgw_status(pkt.rf_chain, TX_STATUS, &tx_status); /* get TX status */
     } while (tx_status != TX_FREE);
    printf("TX done\n");

     /* Stop the gateway */
    x = lgw_stop();
    if (x != 0) {
        printf("ERROR: failed to stop the gateway\n");
        return -1;
    }

    printf("=========== Test End ===========\n");
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

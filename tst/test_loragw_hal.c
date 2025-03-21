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

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(void)
{
    /* Port COM */
    const char com_path[] = "COM7";

    int x;

    x = loragw_default_config(com_path);
    if (x != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure the concentrator \n");
    } else {
        printf("\nINFO: concentrator configuration done");
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
    struct lgw_pkt_rx_s rxpkt[16];
    uint32_t nb_loop = 0;
    uint32_t nb_pkt = 0;
    uint32_t nb_pkt_crc_ok = 0;

    while ((nb_pkt_crc_ok < 5) && (nb_loop < 500)) {
        /* fetch N packets */
        nb_pkt = lgw_receive(16, rxpkt);

        if (nb_pkt == 0) {
            wait_ms(100);
            nb_loop++;
        } else {
            for (uint32_t i = 0; i < nb_pkt; i++) {
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
                for (uint32_t j = 0; j < rxpkt[i].size; j++) {
                    printf("%02X ", rxpkt[i].payload[j]);
                }
                printf("\n");
            }
            printf("Received %d packets (total:%d)\n", nb_pkt, nb_pkt_crc_ok);
        }
    }

    printf( "Nb valid packets received: %d CRC OK\n", nb_pkt_crc_ok );


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

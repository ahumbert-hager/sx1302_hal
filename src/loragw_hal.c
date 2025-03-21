/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    LoRa concentrator Hardware Abstraction Layer

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

#define _GNU_SOURCE     /* needed for qsort_r to be defined */
#include <stdlib.h>     /* qsort_r */

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <string.h>     /* memcpy */
#include <unistd.h>     /* symlink, unlink */
#include <inttypes.h>

#include "loragw_reg.h"
#include "loragw_hal.h"
#include "loragw_aux.h"
#include "loragw_com.h"
#include "loragw_sx1250.h"
#include "loragw_sx1261.h"
#include "loragw_sx1302.h"
#include "loragw_sx1302_timestamp.h"

/* -------------------------------------------------------------------------- */
/* --- DEBUG CONSTANTS ------------------------------------------------------ */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#if DEBUG_HAL == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a<b;++a) fprintf(stdout,"%x.",c[a]);fprintf(stdout,"end\n")
    #define CHECK_NULL(a)                 if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return LGW_HAL_ERROR;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define DEBUG_ARRAY(a,b,c)            for(a=0;a!=0;){}
    #define CHECK_NULL(a)                 if(a==NULL){return LGW_HAL_ERROR;}
#endif

#define TRACE()             fprintf(stderr, "@ %s %d\n", __FUNCTION__, __LINE__);

#define CONTEXT_STARTED         lgw_context.is_started
#define CONTEXT_COM_PATH        lgw_context.board_cfg.com_path
#define CONTEXT_LWAN_PUBLIC     lgw_context.board_cfg.lorawan_public
#define CONTEXT_BOARD           lgw_context.board_cfg
#define CONTEXT_RF_CHAIN        lgw_context.rf_chain_cfg
#define CONTEXT_IF_CHAIN        lgw_context.if_chain_cfg
#define CONTEXT_DEMOD           lgw_context.demod_cfg
#define CONTEXT_LORA_SERVICE    lgw_context.lora_service_cfg
#define CONTEXT_FSK             lgw_context.fsk_cfg
#define CONTEXT_SX1261          lgw_context.sx1261_cfg

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS & TYPES -------------------------------------------- */

#define FW_VERSION_AGC_SX1250   10 /* Expected version of AGC firmware for sx1250 based gateway */
                                   /* v10 is same as v6 with improved channel check time for LBT */
#define FW_VERSION_AGC_SX125X   6  /* Expected version of AGC firmware for sx1255/sx1257 based gateway */
#define FW_VERSION_ARB          2  /* Expected version of arbiter firmware */

/* Useful bandwidth of SX125x radios to consider depending on channel bandwidth */
/* Note: the below values come from lab measurements. For any question, please contact Semtech support */
#define LGW_RF_RX_BANDWIDTH_125KHZ  1600000     /* for 125KHz channels */
#define LGW_RF_RX_BANDWIDTH_250KHZ  1600000     /* for 250KHz channels */
#define LGW_RF_RX_BANDWIDTH_500KHZ  1600000     /* for 500KHz channels */

#define LGW_RF_RX_FREQ_MIN          100E6
#define LGW_RF_RX_FREQ_MAX          1E9

/* Version string, used to identify the library version/options once compiled */
const char lgw_version_string[] = "Version: " LIBLORAGW_VERSION ";";

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

#include "arb_fw.var"           /* text_arb_sx1302_13_Nov_3 */
#include "agc_fw_sx1250.var"    /* text_agc_sx1250_05_Juillet_2019_3 */

/*
The following static variable holds the gateway configuration provided by the
user that need to be propagated in the drivers.

Parameters validity and coherency is verified by the _setconf functions and
the _start and _send functions assume they are valid.
*/
static lgw_context_t lgw_context = {
    .is_started = false,
    .board_cfg.com_path = "/dev/spidev0.0",
    .board_cfg.lorawan_public = true,
    .board_cfg.clksrc = 0,
    .rf_chain_cfg = {{0}},
    .if_chain_cfg = {{0}},
    .demod_cfg = {
        .multisf_datarate = LGW_MULTI_SF_EN
    },
    .lora_service_cfg = {
        .enable = 0,    /* not used, handled by if_chain_cfg */
        .rf_chain = 0,  /* not used, handled by if_chain_cfg */
        .freq_hz = 0,   /* not used, handled by if_chain_cfg */
        .bandwidth = BW_250KHZ,
        .datarate = DR_LORA_SF7,
        .implicit_hdr = false,
        .implicit_payload_length = 0,
        .implicit_crc_en = 0,
        .implicit_coderate = 0
    },
    .fsk_cfg = {
        .enable = 0,    /* not used, handled by if_chain_cfg */
        .rf_chain = 0,  /* not used, handled by if_chain_cfg */
        .freq_hz = 0,   /* not used, handled by if_chain_cfg */
        .bandwidth = BW_125KHZ,
        .datarate = 50000,
        .sync_word_size = 3,
        .sync_word = 0xC194C1
    },
    .sx1261_cfg = {
        .enable = false,
        .spi_path = "/dev/spidev0.1",
        .rssi_offset = 0,
    }
};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DECLARATION ---------------------------------------- */

int32_t lgw_sf_getval(int x);
int32_t lgw_bw_getval(int x);

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS DEFINITION ----------------------------------------- */

int32_t lgw_bw_getval(int x) {
    switch (x) {
        case BW_500KHZ: return 500000;
        case BW_250KHZ: return 250000;
        case BW_125KHZ: return 125000;
        default: return -1;
    }
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int32_t lgw_sf_getval(int x) {
    switch (x) {
        case DR_LORA_SF5: return 5;
        case DR_LORA_SF6: return 6;
        case DR_LORA_SF7: return 7;
        case DR_LORA_SF8: return 8;
        case DR_LORA_SF9: return 9;
        case DR_LORA_SF10: return 10;
        case DR_LORA_SF11: return 11;
        case DR_LORA_SF12: return 12;
        default: return -1;
    }
}

/* -------------------------------------------------------------------------- */
/* --- PUBLIC FUNCTIONS DEFINITION ------------------------------------------ */

int lgw_board_setconf(struct lgw_conf_board_s * conf) {
    CHECK_NULL(conf);

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    CONTEXT_LWAN_PUBLIC = conf->lorawan_public;
    CONTEXT_BOARD.clksrc = conf->clksrc;
    strncpy(CONTEXT_COM_PATH, conf->com_path, sizeof CONTEXT_COM_PATH);
    CONTEXT_COM_PATH[sizeof CONTEXT_COM_PATH - 1] = '\0'; /* ensure string termination */

    DEBUG_PRINTF("Note: board configuration: com_type: %s, com_path: %s, lorawan_public:%d, clksrc:%d,\n",   "USB",
                                                                                                                            CONTEXT_COM_PATH,
                                                                                                                            CONTEXT_LWAN_PUBLIC,
                                                                                                                            CONTEXT_BOARD.clksrc);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxrf_setconf(uint8_t rf_chain, struct lgw_conf_rxrf_s * conf) {
    CHECK_NULL(conf);

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    if (conf->enable == false) {
        /* nothing to do */
        DEBUG_PRINTF("Note: rf_chain %d disabled\n", rf_chain);
        return LGW_HAL_SUCCESS;
    }

    /* check input range (segfault prevention) */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* check if the radio central frequency is valid */
    if ((conf->freq_hz < LGW_RF_RX_FREQ_MIN) || (conf->freq_hz > LGW_RF_RX_FREQ_MAX)) {
        DEBUG_PRINTF("ERROR: NOT A VALID RADIO CENTER FREQUENCY, PLEASE CHECK IF IT HAS BEEN GIVEN IN HZ (%u)\n", conf->freq_hz);
        return LGW_HAL_ERROR;
    }

    /* set internal config according to parameters */
    CONTEXT_RF_CHAIN[rf_chain].enable = conf->enable;
    CONTEXT_RF_CHAIN[rf_chain].freq_hz = conf->freq_hz;
    CONTEXT_RF_CHAIN[rf_chain].rssi_offset = conf->rssi_offset;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_a = conf->rssi_tcomp.coeff_a;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_b = conf->rssi_tcomp.coeff_b;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_c = conf->rssi_tcomp.coeff_c;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_d = conf->rssi_tcomp.coeff_d;
    CONTEXT_RF_CHAIN[rf_chain].rssi_tcomp.coeff_e = conf->rssi_tcomp.coeff_e;
    CONTEXT_RF_CHAIN[rf_chain].tx_enable = conf->tx_enable;
    CONTEXT_RF_CHAIN[rf_chain].single_input_mode = conf->single_input_mode;

    DEBUG_PRINTF("Note: rf_chain %d configuration; en:%d freq:%d rssi_offset:%f tx_enable:%d single_input_mode:%d\n",  rf_chain,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].enable,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].freq_hz,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].rssi_offset,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].type,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].tx_enable,
                                                                                                                CONTEXT_RF_CHAIN[rf_chain].single_input_mode);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_rxif_setconf(uint8_t if_chain, struct lgw_conf_rxif_s * conf) {
    int32_t bw_hz;
    uint32_t rf_rx_bandwidth;

    CHECK_NULL(conf);

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("ERROR: CONCENTRATOR IS RUNNING, STOP IT BEFORE TOUCHING CONFIGURATION\n");
        return LGW_HAL_ERROR;
    }

    /* check input range (segfault prevention) */
    if (if_chain >= LGW_IF_CHAIN_NB) {
        DEBUG_PRINTF("ERROR: %d NOT A VALID IF_CHAIN NUMBER\n", if_chain);
        return LGW_HAL_ERROR;
    }

    /* if chain is disabled, don't care about most parameters */
    if (conf->enable == false) {
        CONTEXT_IF_CHAIN[if_chain].enable = false;
        CONTEXT_IF_CHAIN[if_chain].freq_hz = 0;
        DEBUG_PRINTF("Note: if_chain %d disabled\n", if_chain);
        return LGW_HAL_SUCCESS;
    }

    /* check 'general' parameters */
    if (sx1302_get_ifmod_config(if_chain) == IF_UNDEFINED) {
        DEBUG_PRINTF("ERROR: IF CHAIN %d NOT CONFIGURABLE\n", if_chain);
    }
    if (conf->rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: INVALID RF_CHAIN TO ASSOCIATE WITH A LORA_STD IF CHAIN\n");
        return LGW_HAL_ERROR;
    }
    /* check if IF frequency is optimal based on channel and radio bandwidths */
    switch (conf->bandwidth) {
        case BW_250KHZ:
            rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_250KHZ; /* radio bandwidth */
            break;
        case BW_500KHZ:
            rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_500KHZ; /* radio bandwidth */
            break;
        default:
            /* For 125KHz and below */
            rf_rx_bandwidth = LGW_RF_RX_BANDWIDTH_125KHZ; /* radio bandwidth */
            break;
    }
    bw_hz = lgw_bw_getval(conf->bandwidth); /* channel bandwidth */
    if ((conf->freq_hz + ((bw_hz==-1)?LGW_REF_BW:bw_hz)/2) > ((int32_t)rf_rx_bandwidth/2)) {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO HIGH\n", conf->freq_hz);
        return LGW_HAL_ERROR;
    } else if ((conf->freq_hz - ((bw_hz==-1)?LGW_REF_BW:bw_hz)/2) < -((int32_t)rf_rx_bandwidth/2)) {
        DEBUG_PRINTF("ERROR: IF FREQUENCY %d TOO LOW\n", conf->freq_hz);
        return LGW_HAL_ERROR;
    }

    /* check parameters according to the type of IF chain + modem,
    fill default if necessary, and commit configuration if everything is OK */
    switch (sx1302_get_ifmod_config(if_chain)) {
        case IF_LORA_STD:
            /* fill default parameters if needed */
            if (conf->bandwidth == BW_UNDEFINED) {
                conf->bandwidth = BW_250KHZ;
            }
            if (conf->datarate == DR_UNDEFINED) {
                conf->datarate = DR_LORA_SF7;
            }
            /* check BW & DR */
            if (!IS_LORA_BW(conf->bandwidth)) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_STD IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if (!IS_LORA_DR(conf->datarate)) {
                DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY LORA_STD IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            CONTEXT_IF_CHAIN[if_chain].enable = conf->enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf->rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf->freq_hz;
            CONTEXT_LORA_SERVICE.bandwidth = conf->bandwidth;
            CONTEXT_LORA_SERVICE.datarate = conf->datarate;
            CONTEXT_LORA_SERVICE.implicit_hdr = conf->implicit_hdr;
            CONTEXT_LORA_SERVICE.implicit_payload_length = conf->implicit_payload_length;
            CONTEXT_LORA_SERVICE.implicit_crc_en   = conf->implicit_crc_en;
            CONTEXT_LORA_SERVICE.implicit_coderate = conf->implicit_coderate;

            DEBUG_PRINTF("Note: LoRa 'std' if_chain %d configuration; en:%d freq:%d bw:%d dr:%d\n", if_chain,
                                                                                                    CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                                    CONTEXT_IF_CHAIN[if_chain].freq_hz,
                                                                                                    CONTEXT_LORA_SERVICE.bandwidth,
                                                                                                    CONTEXT_LORA_SERVICE.datarate);
            break;

        case IF_LORA_MULTI:
            /* fill default parameters if needed */
            if (conf->bandwidth == BW_UNDEFINED) {
                conf->bandwidth = BW_125KHZ;
            }
            if (conf->datarate == DR_UNDEFINED) {
                conf->datarate = DR_LORA_SF7;
            }
            /* check BW & DR */
            if (conf->bandwidth != BW_125KHZ) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if (!IS_LORA_DR(conf->datarate)) {
                DEBUG_MSG("ERROR: DATARATE(S) NOT SUPPORTED BY LORA_MULTI IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            CONTEXT_IF_CHAIN[if_chain].enable = conf->enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf->rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf->freq_hz;

            DEBUG_PRINTF("Note: LoRa 'multi' if_chain %d configuration; en:%d freq:%d\n",   if_chain,
                                                                                            CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                            CONTEXT_IF_CHAIN[if_chain].freq_hz);
            break;

        case IF_FSK_STD:
            /* fill default parameters if needed */
            if (conf->bandwidth == BW_UNDEFINED) {
                conf->bandwidth = BW_250KHZ;
            }
            if (conf->datarate == DR_UNDEFINED) {
                conf->datarate = 64000; /* default datarate */
            }
            /* check BW & DR */
            if(!IS_FSK_BW(conf->bandwidth)) {
                DEBUG_MSG("ERROR: BANDWIDTH NOT SUPPORTED BY FSK IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            if(!IS_FSK_DR(conf->datarate)) {
                DEBUG_MSG("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
                return LGW_HAL_ERROR;
            }
            /* set internal configuration  */
            CONTEXT_IF_CHAIN[if_chain].enable = conf->enable;
            CONTEXT_IF_CHAIN[if_chain].rf_chain = conf->rf_chain;
            CONTEXT_IF_CHAIN[if_chain].freq_hz = conf->freq_hz;
            CONTEXT_FSK.bandwidth = conf->bandwidth;
            CONTEXT_FSK.datarate = conf->datarate;
            if (conf->sync_word > 0) {
                CONTEXT_FSK.sync_word_size = conf->sync_word_size;
                CONTEXT_FSK.sync_word = conf->sync_word;
            }
            DEBUG_PRINTF("Note: FSK if_chain %d configuration; en:%d freq:%d bw:%d dr:%d (%d real dr) sync:0x%0*" PRIu64 "\n", if_chain,
                                                                                                                        CONTEXT_IF_CHAIN[if_chain].enable,
                                                                                                                        CONTEXT_IF_CHAIN[if_chain].freq_hz,
                                                                                                                        CONTEXT_FSK.bandwidth,
                                                                                                                        CONTEXT_FSK.datarate,
                                                                                                                        LGW_XTAL_FREQU/(LGW_XTAL_FREQU/CONTEXT_FSK.datarate),
                                                                                                                        2*CONTEXT_FSK.sync_word_size,
                                                                                                                        CONTEXT_FSK.sync_word);
            break;

        default:
            DEBUG_PRINTF("ERROR: IF CHAIN %d TYPE NOT SUPPORTED\n", if_chain);
            return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_demod_setconf(struct lgw_conf_demod_s * conf) {
    CHECK_NULL(conf);

    CONTEXT_DEMOD.multisf_datarate = conf->multisf_datarate;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_start(void) {
    int i, err;
    uint8_t fw_version_agc;

    if (CONTEXT_STARTED == true) {
        DEBUG_MSG("Note: LoRa concentrator already started, restarting it now\n");
    }

    err = lgw_connect(CONTEXT_COM_PATH);
    if (err == LGW_REG_ERROR) {
        DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
        return LGW_HAL_ERROR;
    }

    /* Set all GPIOs to 0 */
    err = sx1302_set_gpio(0x00);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to set all GPIOs to 0\n");
        return LGW_HAL_ERROR;
    }

    /* Calibrate radios */
    err = sx1302_radio_calibrate(&CONTEXT_RF_CHAIN[0], CONTEXT_BOARD.clksrc);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: radio calibration failed\n");
        return LGW_HAL_ERROR;
    }

    /* Setup radios for RX */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        if (CONTEXT_RF_CHAIN[i].enable == true) {
            /* Reset the radio */
            err = sx1302_radio_reset(i);
            if (err != LGW_REG_SUCCESS) {
                printf("ERROR: failed to reset radio %d\n", i);
                return LGW_HAL_ERROR;
            }

            /* Setup the radio */
            err = sx1250_setup(i, CONTEXT_RF_CHAIN[i].freq_hz, CONTEXT_RF_CHAIN[i].single_input_mode);
            if (err != LGW_REG_SUCCESS) {
                printf("ERROR: failed to setup radio %d\n", i);
                return LGW_HAL_ERROR;
            }

            /* Set radio mode */
            err = sx1302_radio_set_mode(i);
            if (err != LGW_REG_SUCCESS) {
                printf("ERROR: failed to set mode for radio %d\n", i);
                return LGW_HAL_ERROR;
            }
        }
    }

    /* Select the radio which provides the clock to the sx1302 */
    err = sx1302_radio_clock_select(CONTEXT_BOARD.clksrc);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to get clock from radio %u\n", CONTEXT_BOARD.clksrc);
        return LGW_HAL_ERROR;
    }

    /* Release host control on radio (will be controlled by AGC) */
    err = sx1302_radio_host_ctrl(false);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to release control over radios\n");
        return LGW_HAL_ERROR;
    }

    /* Basic initialization of the sx1302 */
    err = sx1302_init();
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to initialize SX1302\n");
        return LGW_HAL_ERROR;
    }

    /* Configure PA/LNA LUTs */
    err = sx1302_pa_lna_lut_configure();
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure SX1302 PA/LNA LUT\n");
        return LGW_HAL_ERROR;
    }

    /* Configure Radio FE */
    err = sx1302_radio_fe_configure();
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure SX1302 radio frontend\n");
        return LGW_HAL_ERROR;
    }

    /* Configure the Channelizer */
    err = sx1302_channelizer_configure(CONTEXT_IF_CHAIN, false);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure SX1302 channelizer\n");
        return LGW_HAL_ERROR;
    }

    /* configure LoRa 'multi-sf' modems */
    err = sx1302_lora_correlator_configure(CONTEXT_IF_CHAIN, &(CONTEXT_DEMOD));
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure SX1302 LoRa modem correlators\n");
        return LGW_HAL_ERROR;
    }
    err = sx1302_lora_modem_configure(CONTEXT_RF_CHAIN[0].freq_hz);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure SX1302 LoRa modems\n");
        return LGW_HAL_ERROR;
    }

    /* configure LoRa 'single-sf' modem */
    if (CONTEXT_IF_CHAIN[8].enable == true) {
        err = sx1302_lora_service_correlator_configure(&(CONTEXT_LORA_SERVICE));
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: failed to configure SX1302 LoRa Service modem correlators\n");
            return LGW_HAL_ERROR;
        }
        err = sx1302_lora_service_modem_configure(&(CONTEXT_LORA_SERVICE), CONTEXT_RF_CHAIN[0].freq_hz);
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: failed to configure SX1302 LoRa Service modem\n");
            return LGW_HAL_ERROR;
        }
    }

    /* configure FSK modem */
    if (CONTEXT_IF_CHAIN[9].enable == true) {
        err = sx1302_fsk_configure(&(CONTEXT_FSK));
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: failed to configure SX1302 FSK modem\n");
            return LGW_HAL_ERROR;
        }
    }

    /* configure syncword */
    err = sx1302_lora_syncword(CONTEXT_LWAN_PUBLIC, CONTEXT_LORA_SERVICE.datarate);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure SX1302 LoRa syncword\n");
        return LGW_HAL_ERROR;
    }

    /* enable demodulators - to be done before starting AGC/ARB */
    err = sx1302_modem_enable();
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to enable SX1302 modems\n");
        return LGW_HAL_ERROR;
    }

    /* Load AGC firmware */
    DEBUG_MSG("Loading AGC fw for sx1250\n");
    err = sx1302_agc_load_firmware(agc_firmware_sx1250);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to load AGC firmware for sx1250\n");
        return LGW_HAL_ERROR;
    }
    fw_version_agc = FW_VERSION_AGC_SX1250;

    err = sx1302_agc_start(fw_version_agc, SX1302_AGC_RADIO_GAIN_AUTO, SX1302_AGC_RADIO_GAIN_AUTO);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to start AGC firmware\n");
        return LGW_HAL_ERROR;
    }

    /* Load ARB firmware */
    DEBUG_MSG("Loading ARB fw\n");
    err = sx1302_arb_load_firmware(arb_firmware);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to load ARB firmware\n");
        return LGW_HAL_ERROR;
    }
    err = sx1302_arb_start(FW_VERSION_ARB);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to start ARB firmware\n");
        return LGW_HAL_ERROR;
    }

    /* static TX configuration */
    err = sx1302_tx_configure();
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to configure SX1302 TX path\n");
        return LGW_HAL_ERROR;
    }

    /* enable GPS */
    err = sx1302_gps_enable(true);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to enable GPS on sx1302\n");
        return LGW_HAL_ERROR;
    }

    /* Connect to the external sx1261 for LBT or Spectral Scan */
    if (CONTEXT_SX1261.enable == true) {
        err = sx1261_load_pram();
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: failed to patch sx1261 radio for LBT/Spectral Scan\n");
            return LGW_HAL_ERROR;
        }

        err = sx1261_calibrate(CONTEXT_RF_CHAIN[0].freq_hz);
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: failed to calibrate sx1261 radio\n");
            return LGW_HAL_ERROR;
        }

        err = sx1261_setup();
        if (err != LGW_REG_SUCCESS) {
            printf("ERROR: failed to setup sx1261 radio\n");
            return LGW_HAL_ERROR;
        }
    }

    /* Set CONFIG_DONE GPIO to 1 (turn on the corresponding LED) */
    err = sx1302_set_gpio(0x01);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: failed to set CONFIG_DONE GPIO\n");
        return LGW_HAL_ERROR;
    }

    /* set hal state */
    CONTEXT_STARTED = true;

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_stop(void) {
    int i, x, err = LGW_HAL_SUCCESS;

    if (CONTEXT_STARTED == false) {
        DEBUG_MSG("Note: LoRa concentrator was not started...\n");
        return LGW_HAL_SUCCESS;
    }

    /* Abort current TX if needed */
    for (i = 0; i < LGW_RF_CHAIN_NB; i++) {
        DEBUG_PRINTF("INFO: aborting TX on chain %u\n", i);
        x = lgw_abort_tx(i);
        if (x != LGW_HAL_SUCCESS) {
            printf("WARNING: failed to get abort TX on chain %u\n", i);
            err = LGW_HAL_ERROR;
        }
    }

    DEBUG_MSG("INFO: Disconnecting\n");
    x = lgw_disconnect();
    if (x != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to disconnect concentrator\n");
        err = LGW_HAL_ERROR;
    }

    CONTEXT_STARTED = false;

    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_receive(uint8_t max_pkt, struct lgw_pkt_rx_s *pkt_data) {
    int res;
    uint8_t nb_pkt_fetched = 0;
    uint8_t nb_pkt_found = 0;
    uint8_t nb_pkt_left = 0;
    float current_temperature = 0.0, rssi_temperature_offset = 0.0;

    /* Get packets from SX1302, if any */
    res = sx1302_fetch(&nb_pkt_fetched);
    if (res != LGW_REG_SUCCESS) {
        printf("ERROR: failed to fetch packets from SX1302\n");
        return LGW_HAL_ERROR;
    }

    /* Update internal counter */
    /* WARNING: this needs to be called regularly by the upper layer */
    res = sx1302_update();
    if (res != LGW_REG_SUCCESS) {
        return LGW_HAL_ERROR;
    }

    /* Exit now if no packet fetched */
    if (nb_pkt_fetched == 0) {
        return 0;
    }
    if (nb_pkt_fetched > max_pkt) {
        nb_pkt_left = nb_pkt_fetched - max_pkt;
        printf("WARNING: not enough space allocated, fetched %d packet(s), %d will be left in RX buffer\n", nb_pkt_fetched, nb_pkt_left);
    }

    /* Apply RSSI temperature compensation */
    res = lgw_get_temperature(&current_temperature);
    if (res != 0) {
        printf("ERROR: failed to get current temperature\n");
        return LGW_HAL_ERROR;
    }

    /* Iterate on the RX buffer to get parsed packets */
    for (nb_pkt_found = 0; nb_pkt_found < ((nb_pkt_fetched <= max_pkt) ? nb_pkt_fetched : max_pkt); nb_pkt_found++) {
        /* Get packet and move to next one */
        res = sx1302_parse(&lgw_context, &pkt_data[nb_pkt_found]);
        if (res == LGW_REG_WARNING) {
            printf("WARNING: parsing error on packet %d, discarding fetched packets\n", nb_pkt_found);
            return LGW_HAL_SUCCESS;
        } else if (res == LGW_REG_ERROR) {
            printf("ERROR: fatal parsing error on packet %d, aborting...\n", nb_pkt_found);
            return LGW_HAL_ERROR;
        }

        /* Appli RSSI offset calibrated for the board */
        pkt_data[nb_pkt_found].rssic += CONTEXT_RF_CHAIN[pkt_data[nb_pkt_found].rf_chain].rssi_offset;
        pkt_data[nb_pkt_found].rssis += CONTEXT_RF_CHAIN[pkt_data[nb_pkt_found].rf_chain].rssi_offset;

        rssi_temperature_offset = sx1302_rssi_get_temperature_offset(&CONTEXT_RF_CHAIN[pkt_data[nb_pkt_found].rf_chain].rssi_tcomp, current_temperature);
        pkt_data[nb_pkt_found].rssic += rssi_temperature_offset;
        pkt_data[nb_pkt_found].rssis += rssi_temperature_offset;
        DEBUG_PRINTF("INFO: RSSI temperature offset applied: %.3f dB (current temperature %.1f C)\n", rssi_temperature_offset, current_temperature);
    }

    DEBUG_PRINTF("INFO: nb pkt found:%u left:%u\n", nb_pkt_found, nb_pkt_left);

    return nb_pkt_found;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_send(struct lgw_pkt_tx_s * pkt_data) {
    int err;

    /* check if the concentrator is running */
    if (CONTEXT_STARTED == false) {
        printf("ERROR: CONCENTRATOR IS NOT RUNNING, START IT BEFORE SENDING\n");
        return LGW_HAL_ERROR;
    }

    CHECK_NULL(pkt_data);

    /* check input range (segfault prevention) */
    if (pkt_data->rf_chain >= LGW_RF_CHAIN_NB) {
        printf("ERROR: INVALID RF_CHAIN TO SEND PACKETS\n");
        return LGW_HAL_ERROR;
    }

    /* check input variables */
    if (CONTEXT_RF_CHAIN[pkt_data->rf_chain].tx_enable == false) {
        printf("ERROR: SELECTED RF_CHAIN IS DISABLED FOR TX ON SELECTED BOARD\n");
        return LGW_HAL_ERROR;
    }
    if (CONTEXT_RF_CHAIN[pkt_data->rf_chain].enable == false) {
        printf("ERROR: SELECTED RF_CHAIN IS DISABLED\n");
        return LGW_HAL_ERROR;
    }
    if (!IS_TX_MODE(pkt_data->tx_mode)) {
        printf("ERROR: TX_MODE NOT SUPPORTED\n");
        return LGW_HAL_ERROR;
    }
    if (pkt_data->modulation == MOD_LORA) {
        if (!IS_LORA_BW(pkt_data->bandwidth)) {
            printf("ERROR: BANDWIDTH NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_DR(pkt_data->datarate)) {
            printf("ERROR: DATARATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (!IS_LORA_CR(pkt_data->coderate)) {
            printf("ERROR: CODERATE NOT SUPPORTED BY LORA TX\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data->size > 255) {
            printf("ERROR: PAYLOAD LENGTH TOO BIG FOR LORA TX\n");
            return LGW_HAL_ERROR;
        }
    } else if (pkt_data->modulation == MOD_FSK) {
        if((pkt_data->f_dev < 1) || (pkt_data->f_dev > 200)) {
            printf("ERROR: TX FREQUENCY DEVIATION OUT OF ACCEPTABLE RANGE\n");
            return LGW_HAL_ERROR;
        }
        if(!IS_FSK_DR(pkt_data->datarate)) {
            printf("ERROR: DATARATE NOT SUPPORTED BY FSK IF CHAIN\n");
            return LGW_HAL_ERROR;
        }
        if (pkt_data->size > 255) {
            printf("ERROR: PAYLOAD LENGTH TOO BIG FOR FSK TX\n");
            return LGW_HAL_ERROR;
        }
    } else if (pkt_data->modulation == MOD_CW) {
        /* do nothing */
    } else {
        printf("ERROR: INVALID TX MODULATION\n");
        return LGW_HAL_ERROR;
    }

    /* Send the TX request to the concentrator */
    err = sx1302_send(CONTEXT_LWAN_PUBLIC, &CONTEXT_FSK, pkt_data);
    if (err != LGW_REG_SUCCESS) {
        printf("ERROR: %s: Failed to send packet\n", __FUNCTION__);
        return LGW_HAL_ERROR;
    }

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_status(uint8_t rf_chain, uint8_t select, uint8_t *code) {

    /* check input variables */
    CHECK_NULL(code);
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* Get status */
    if (select == TX_STATUS) {
        if (CONTEXT_STARTED == false) {
            *code = TX_OFF;
        } else {
            *code = sx1302_tx_status(rf_chain);
        }
    } else if (select == RX_STATUS) {
        if (CONTEXT_STARTED == false) {
            *code = RX_OFF;
        } else {
            *code = sx1302_rx_status(rf_chain);
        }
    } else {
        DEBUG_MSG("ERROR: SELECTION INVALID, NO STATUS TO RETURN\n");
        return LGW_HAL_ERROR;
    }
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_abort_tx(uint8_t rf_chain) {
    int err;
    /* check input variables */
    if (rf_chain >= LGW_RF_CHAIN_NB) {
        DEBUG_MSG("ERROR: NOT A VALID RF_CHAIN NUMBER\n");
        return LGW_HAL_ERROR;
    }

    /* Abort current TX */
    err = sx1302_tx_abort(rf_chain);
    return err;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_trigcnt(uint32_t* trig_cnt_us) {
    CHECK_NULL(trig_cnt_us);

    *trig_cnt_us = sx1302_timestamp_counter(true);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_instcnt(uint32_t* inst_cnt_us) {
    CHECK_NULL(inst_cnt_us);

    *inst_cnt_us = sx1302_timestamp_counter(false);

    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_eui(uint64_t* eui) {
    CHECK_NULL(eui);

    if (sx1302_get_eui(eui) != LGW_REG_SUCCESS) {
        return LGW_HAL_ERROR;
    }
    return LGW_HAL_SUCCESS;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

int lgw_get_temperature(float* temperature) {
    CHECK_NULL(temperature);
    return lgw_com_get_temperature(temperature);
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

const char* lgw_version_info() {
    return lgw_version_string;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

uint32_t lgw_time_on_air(const struct lgw_pkt_tx_s *packet) {
    double t_fsk;
    uint32_t toa_ms, toa_us;

    DEBUG_PRINTF(" --- %s\n", "IN");

    if (packet == NULL) {
        printf("ERROR: Failed to compute time on air, wrong parameter\n");
        return 0;
    }

    if (packet->modulation == MOD_LORA) {
        toa_us = lora_packet_time_on_air(packet->bandwidth, packet->datarate, packet->coderate, packet->preamble, packet->no_header, packet->no_crc, packet->size, NULL, NULL, NULL);
        toa_ms = (uint32_t)( (double)toa_us / 1000.0 + 0.5 );
        DEBUG_PRINTF("INFO: LoRa packet ToA: %u ms\n", toa_ms);
    } else if (packet->modulation == MOD_FSK) {
        /* PREAMBLE + SYNC_WORD + PKT_LEN + PKT_PAYLOAD + CRC
                PREAMBLE: default 5 bytes
                SYNC_WORD: default 3 bytes
                PKT_LEN: 1 byte (variable length mode)
                PKT_PAYLOAD: x bytes
                CRC: 0 or 2 bytes
        */
        t_fsk = (8 * (double)(packet->preamble + CONTEXT_FSK.sync_word_size + 1 + packet->size + ((packet->no_crc == true) ? 0 : 2)) / (double)packet->datarate) * 1E3;

        /* Duration of packet */
        toa_ms = (uint32_t)t_fsk + 1; /* add margin for rounding */
    } else {
        toa_ms = 0;
        printf("ERROR: Cannot compute time on air for this packet, unsupported modulation (0x%02X)\n", packet->modulation);
    }

    DEBUG_PRINTF(" --- %s\n", "OUT");

    return toa_ms;
}

int loragw_default_config(const char * com_path)
{
    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_rxif_s ifconf;

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
    rfconf.tx_enable = true;
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
    for (int i = 0; i < 8; i++) {
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
    ifconf.rf_chain = channel_rfchain_mode0[8];
    ifconf.freq_hz = channel_if_mode0[8];
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
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

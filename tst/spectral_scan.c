/*
  ______                              _
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2020 Semtech

Description:
    Spectral Scan Utility

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
#include <inttypes.h>   /* PRIx64, PRIu64... */
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>     /* sigaction */
#include <getopt.h>     /* getopt_long */

#include "loragw_hal.h"
#include "loragw_aux.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE CONSTANTS ---------------------------------------------------- */

#define COM_TYPE_DEFAULT    LGW_COM_USB
#define COM_PATH_DEFAULT    "/dev/spidev0.0"
#define SX1261_PATH_DEFAULT "/dev/spidev0.1"

#define DEFAULT_CLK_SRC     0
#define DEFAULT_RADIO_TYPE  LGW_RADIO_TYPE_SX1250
#define DEFAULT_FREQ_HZ     863100000U
#define DEFAULT_NB_CHAN     35
#define DEFAULT_NB_SCAN     2000
#define DEFAULT_RSSI_OFFSET -11 /* RSSI offset of SX1261 */

#define DEFAULT_LOG_NAME    "rssi_histogram"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* describe command line options */
void usage(void) {
    printf("Library version information: %s\n", lgw_version_info());
    printf("Available options:\n");
    printf(" -h         Print this help\n");
    printf(" -u         Set COM type as USB (default is SPI)\n");
    printf(" -d [path]  Path to the main COM interface\n");
    printf("            => default path: " COM_PATH_DEFAULT "\n");
    printf(" -D [path]  Path to the SX1261 SPI interface (not used for USB)\n");
    printf("            => default path: " SX1261_PATH_DEFAULT "\n");
    printf(" -f <float> Scan start frequency, in MHz\n");
    printf(" -n <uint>  Number of channels to scan\n");
    printf(" -s <uint>  Number of scan points per frequency step [1..65535]\n");
    printf(" -o <int>   RSSI Offset of the sx1261 path, in dB [-127..128]\n");
    printf(" -l <char>  Log file name\n");
}

/* -------------------------------------------------------------------------- */
/* --- MAIN FUNCTION -------------------------------------------------------- */

int main(int argc, char **argv)
{
    int i, j, x;
    unsigned int arg_u;
    double arg_d = 0.0;
    int arg_i;
    char arg_s[64];

    struct lgw_conf_board_s boardconf;
    struct lgw_conf_rxrf_s rfconf;
    struct lgw_conf_sx1261_s sx1261conf;

    /* COM interface */
    const char com_path_default[] = COM_PATH_DEFAULT;
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = COM_TYPE_DEFAULT;
    const char sx1261_path_default[] = SX1261_PATH_DEFAULT;
    const char * sx1261_path = sx1261_path_default;

    /* Spectral Scan */
    uint32_t freq_hz = DEFAULT_FREQ_HZ;
    uint8_t nb_channels = DEFAULT_NB_CHAN;
    uint16_t nb_scan = DEFAULT_NB_SCAN;
    int8_t rssi_offset = DEFAULT_RSSI_OFFSET;
    int16_t levels[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    uint16_t results[LGW_SPECTRAL_SCAN_RESULT_SIZE];
    char log_file_name[64] = DEFAULT_LOG_NAME;
    FILE * log_file = NULL;

    struct timeval tm_start;
    lgw_spectral_scan_status_t status;

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {0, 0, 0, 0}
    };


    printf("==\n");
    printf("== Spectral Scan: freq_hz=%uHz, nb_channels=%u, nb_scan=%u, rssi_offset=%ddB\n", freq_hz, nb_channels, nb_scan, rssi_offset);
    printf("==\n");

    /* Configure the gateway */
    memset(&boardconf, 0, sizeof boardconf);
    boardconf.lorawan_public = true;
    boardconf.clksrc = DEFAULT_CLK_SRC;
    boardconf.full_duplex = false;
    boardconf.com_type = com_type;
    strncpy(boardconf.com_path, com_path, sizeof boardconf.com_path);
    boardconf.com_path[sizeof boardconf.com_path - 1] = '\0'; /* ensure string termination */
    if (lgw_board_setconf(&boardconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure board\n");
        return EXIT_FAILURE;
    }

    memset(&rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 867500000; /* dummy */
    rfconf.type = DEFAULT_RADIO_TYPE;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(0, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 0\n");
        return EXIT_FAILURE;
    }

    memset(&rfconf, 0, sizeof rfconf);
    rfconf.enable = true;
    rfconf.freq_hz = 868500000; /* dummy */
    rfconf.type = DEFAULT_RADIO_TYPE;
    rfconf.tx_enable = false;
    rfconf.single_input_mode = false;
    if (lgw_rxrf_setconf(1, &rfconf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure rxrf 1\n");
        return EXIT_FAILURE;
    }

    /* Configure the sx1261 for spectral scan */
    memset(&sx1261conf, 0, sizeof sx1261conf);
    sx1261conf.enable = true;
    strncpy(sx1261conf.spi_path, sx1261_path, sizeof sx1261conf.spi_path);
    sx1261conf.spi_path[sizeof sx1261conf.spi_path - 1] = '\0'; /* ensure string termination */
    sx1261conf.rssi_offset = rssi_offset;
    sx1261conf.lbt_conf.enable = false;
    if (lgw_sx1261_setconf(&sx1261conf) != LGW_HAL_SUCCESS) {
        printf("ERROR: failed to configure sx1261\n");
        return EXIT_FAILURE;
    }

    /* Start the gateway, initialize sx1261 radio for scanning */
    x = lgw_start();
    if (x != 0) {
        printf("ERROR: failed to start the gateway\n");
        return EXIT_FAILURE;
    }

    /* Launch Spectral Scan on each channels */
    for (j = 0; j < nb_channels; j++) {
        x = lgw_spectral_scan_start(freq_hz, nb_scan);
        if (x != 0) {
            printf("ERROR: spectral scan start failed\n");
            continue;
        }

        /* Wait for scan to be completed */
        timeout_start(&tm_start);
        do {
            /* handle timeout */
            if (timeout_check(tm_start, 2000) != 0) {
                printf("ERROR: %s: TIMEOUT on Spectral Scan\n", __FUNCTION__);
                continue;
            }

            /* get spectral scan status */
            status = LGW_SPECTRAL_SCAN_STATUS_UNKNOWN;
            x = lgw_spectral_scan_get_status(&status);
            if (x != 0) {
                printf("ERROR: spectral scan status failed\n");
                break;
            }
            wait_ms(10);
        } while (status != LGW_SPECTRAL_SCAN_STATUS_COMPLETED && status != LGW_SPECTRAL_SCAN_STATUS_ABORTED);

        if (status == LGW_SPECTRAL_SCAN_STATUS_COMPLETED) {
            memset(levels, 0, sizeof levels);
            memset(results, 0, sizeof results);
            x = lgw_spectral_scan_get_results(levels, results);
            if (x != 0) {
                printf("ERROR: spectral scan get results failed\n");
                continue;
            }

            /* print results */
            printf("%u: ", freq_hz);
            for (i = 0; i < LGW_SPECTRAL_SCAN_RESULT_SIZE; i++) {
                printf("%u ", results[i]);
            }
            printf("\n");

            /* Next frequency to scan */
            freq_hz += 200000; /* 200kHz channels */
        } else if (status == LGW_SPECTRAL_SCAN_STATUS_ABORTED) {
            printf("INFO: spectral scan has been aborted\n");
        } else {
            printf("ERROR: spectral scan status us unexpected 0x%02X\n", status);
        }
    }

    /* Stop the gateway */
    x = lgw_stop();
    if (x != 0) {
        printf("ERROR: failed to stop the gateway\n");
        return EXIT_FAILURE;
    }
    return 0;
}

/* --- EOF ------------------------------------------------------------------ */

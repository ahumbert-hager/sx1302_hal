/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
  (C)2019 Semtech

Description:
    Minimum test program to test the capture RAM block

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

#include <stdio.h>      /* printf */
#include <stdlib.h>
#include <signal.h>     /* sigaction */
#include <getopt.h>     /* getopt_long */

#include "loragw_hal.h"
#include "loragw_com.h"
#include "loragw_reg.h"
#include "loragw_aux.h"
#include "loragw_sx1250.h"
#include "loragw_sx1302.h"

/* -------------------------------------------------------------------------- */
/* --- PRIVATE MACROS ------------------------------------------------------- */

#define DEBUG_MSG(str) fprintf(stdout, str)

#define CAPTURE_RAM_SIZE 0x4000

/* -------------------------------------------------------------------------- */
/* --- PRIVATE VARIABLES ---------------------------------------------------- */
uint32_t sampling_frequency[] = {4e6, 4e6, 4e6, 4e6, 4e6, 4e6, 4e6, 0, 0, 1e6, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 125e3, 8e6, 125e3, 125e3, 125e3, 0, 32e6, 32e6, 0, 32e6, 32e6, 0, 32e6, 32e6, 32e6};

/* -------------------------------------------------------------------------- */
/* --- PRIVATE FUNCTIONS ---------------------------------------------------- */

/* Main program */
int main(void)
{
    int i;
    int32_t val = 0;
    int reg_stat;
    unsigned int arg_u;
    uint8_t capture_source = 0;
    uint16_t period_value = 0;
    int16_t real = 0, imag = 0;
    uint8_t capture_ram_buffer[CAPTURE_RAM_SIZE];

    /* SPI interfaces */
    const char com_path_default[] = "COM7";
    const char * com_path = com_path_default;
    lgw_com_type_t com_type = LGW_COM_USB;

    /* Parameter parsing */
    int option_index = 0;
    static struct option long_options[] = {
        {0, 0, 0, 0}
    };


    /* Initialize memory for capture */
    for (i = 0; i < CAPTURE_RAM_SIZE; i++) {
        capture_ram_buffer[i] = i%256;
    }

    reg_stat = lgw_connect(com_type, com_path);
    if (reg_stat == LGW_REG_ERROR) {
        DEBUG_MSG("ERROR: FAIL TO CONNECT BOARD\n");
        return LGW_HAL_ERROR;
    }

    /* Configure the Capture Ram block */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_ENABLE, 1);    /* Enable Capture RAM */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTUREWRAP, 0);   /* Capture once, and stop when memory is full */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_RAMCONFIG, 0);   /* RAM configuration, 0: 4kx32, 1: 2kx64 */
    fprintf(stdout, "Capture source: %d\n", capture_source);
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_SOURCE_A_SOURCEMUX, capture_source);

    printf("Sampling frequency: %d\n", sampling_frequency[capture_source]);
    if (sampling_frequency[capture_source] != 0) {
        period_value = (32e6/sampling_frequency[capture_source]) - 1;
    } else {
        fprintf(stderr ,"ERROR: Sampling frequency is null\n");
        return -1;
    }

    // fprintf(stdout, "period_value=%04X\n", period_value);
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_0_CAPTUREPERIOD, period_value & 0xFF);  // LSB
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_PERIOD_1_CAPTUREPERIOD, (period_value>>8) & 0xFF); // MSB


    /* Launch capture */
    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTURESTART, 1);

    /* Poll Status.CapComplete */
    do {
        lgw_reg_r(SX1302_REG_CAPTURE_RAM_STATUS_CAPCOMPLETE, &val);

        wait_ms(10);
    } while (val != 1);

    lgw_reg_w(SX1302_REG_CAPTURE_RAM_CAPTURE_CFG_CAPTURESTART, 0);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 1);
    lgw_mem_rb(0, capture_ram_buffer, CAPTURE_RAM_SIZE, false);
    lgw_reg_w(SX1302_REG_COMMON_PAGE_PAGE, 0);

    printf("Data:\n");
    for (i = 0; i < CAPTURE_RAM_SIZE; i += 4) {
        if (((capture_source >= 2) && (capture_source <= 3)) || (capture_source == 9)) {
            real = (int16_t)((((uint16_t)(capture_ram_buffer[i+3]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+2] & 0x00FF));
            imag = (int16_t)((((uint16_t)(capture_ram_buffer[i+1]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+0] & 0x00FF));
            real >>= 4; // 12 bits I
            imag >>= 4; // 12 bits Q
        } else if ((capture_source >= 4) && (capture_source <= 6)) {
            real = (int16_t)((((uint16_t)(capture_ram_buffer[i+3]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+2] & 0x00FF)); // 16 bits I
            imag = (int16_t)((((uint16_t)(capture_ram_buffer[i+1]) << 8) & 0xFF00) + ((uint16_t)capture_ram_buffer[i+0] & 0x00FF)); // 16 bits Q
        } else if ((capture_source >= 10) && (capture_source <= 17)) {
            real = (int8_t)(capture_ram_buffer[i+3]); // 8 bits I
            imag = (int8_t)(capture_ram_buffer[i+1]); // 8 bits Q
        } else {
            real = 0;
            imag = 0;
        }

        if (((capture_source >= 2) && (capture_source <= 6)) || ((capture_source >= 9) && (capture_source <= 17))) {
            fprintf(stdout, "%d", real);
            if (imag >= 0) {
                fprintf(stdout, "+");
            }
            fprintf(stdout, "%di\n", imag);
        } else {
            printf("%02X ", capture_ram_buffer[i]);
        }
    }
    printf("End of Data\n");

    return 0;
}

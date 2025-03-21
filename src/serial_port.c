/*
Copyright (c) 2017 Ahmad Bashar Eter

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "serial_port.h"
#define WINDOWS

#if DEBUG_COM == 1
    #define DEBUG_MSG(str)                fprintf(stdout, str)
    #define DEBUG_PRINTF(fmt, args...)    fprintf(stdout,"%s:%d: "fmt, __FUNCTION__, __LINE__, args)
    #define CHECK_NULL(a)                if(a==NULL){fprintf(stderr,"%s:%d: ERROR: NULL POINTER AS ARGUMENT\n", __FUNCTION__, __LINE__);return -1;}
#else
    #define DEBUG_MSG(str)
    #define DEBUG_PRINTF(fmt, args...)
    #define CHECK_NULL(a)                if(a==NULL){return -1;}
#endif

#ifdef WINDOWS

#include <Windows.h>

#define CP_BOUD_RATE_9600 CBR_9600
#define CP_BOUD_RATE_1155 1155
#define CP_BOUD_RATE_4800 CBR_4800
#define CP_BOUD_RATE_19200 CBR_19200

#define CP_DATA_BITS_5 5
#define CP_DATA_BITS_6 6
#define CP_DATA_BITS_7 7
#define CP_DATA_BITS_8 8

#define CP_STOP_BITS_ONE ONESTOPBIT
#define CP_STOP_BITS_TWO TWOSTOPBITS
#define CP_STOP_BITS_ONE_AND_HALF ONE5STOPBITS

#define CP_PARITY_NOPARITY NOPARITY
#define CP_PARITY_ODD ODDPARITY
#define CP_PARITY_EVEN EVENPARITY
#define CP_PARITY_MARK MARKPARITY
#define CP_PARITY_SPACE SPACEPARITY

typedef HANDLE PORT;

PORT OpenPort(const char * com_path)
{
	HANDLE hComm;
	TCHAR comname[100];
	wsprintf(comname, TEXT("\\\\.\\%s"), com_path);
	hComm = CreateFile(comname,            //port name 
		GENERIC_READ | GENERIC_WRITE, //Read/Write   				 
		0,            // No Sharing                               
		NULL,         // No Security                              
		OPEN_EXISTING,// Open existing port only                     
		0,            // Non Overlapped I/O                           
		NULL);        // Null for Comm Devices
	
	if (hComm == INVALID_HANDLE_VALUE)
		return NULL;
	COMMTIMEOUTS timeouts = { 0 };
	timeouts.ReadIntervalTimeout = 50;
	timeouts.ReadTotalTimeoutConstant = 50;
	timeouts.ReadTotalTimeoutMultiplier = 10;
	timeouts.WriteTotalTimeoutConstant = 50;
	timeouts.WriteTotalTimeoutMultiplier = 10;

	if (SetCommTimeouts(hComm, &timeouts) == FALSE)
		return NULL;

	if (SetCommMask(hComm, EV_RXCHAR) == FALSE)
		return NULL;

	return hComm;
}
void ClosePort(PORT com_port)
{
	CloseHandle(com_port);
}

int SetPortBaudRate(PORT com_port, int rate)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.BaudRate = rate;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int SetPortDataBits(PORT com_port, int bits)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.ByteSize = bits;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int SetPortStopBits(PORT com_port, int bits)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.StopBits = bits;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int SetPortParity(PORT com_port, int parity)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return FALSE;
	dcbSerialParams.Parity = parity;
	Status = SetCommState(com_port, &dcbSerialParams);
	return Status;
}

int GetPortBaudRate(PORT com_port)
{
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.BaudRate;
}
int GetPortDataBits(PORT com_port) {
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.ByteSize;
}
int GetPortStopBits(PORT com_port) {
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.StopBits;
}
int GetPortParity(PORT com_port) {
	DCB dcbSerialParams = { 0 };
	BOOL Status;
	dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
	Status = GetCommState(com_port, &dcbSerialParams);
	if (Status == FALSE)
		return -1;
	return dcbSerialParams.Parity;
}

int SendData(PORT com_port, const unsigned char * data, DWORD aSize)
{
	DWORD  dNoOFBytestoWrite = aSize;
	DWORD  dNoOfBytesWritten;
	BOOL Status = WriteFile(com_port,
				data,
				dNoOFBytestoWrite,
				&dNoOfBytesWritten,
				NULL);
	if (Status == FALSE)
		return -1;
	return dNoOfBytesWritten;
}

DWORD ReceiveData(PORT com_port, unsigned char * data, int len)
{
	DWORD NoBytesRead;
	BOOL Status;
	Status = ReadFile(com_port, data, len, &NoBytesRead, NULL);
	if (Status == FALSE) {
		NoBytesRead = -1;
	}
	return NoBytesRead;
}


//API
PORT hComm = 0;

int serial_open(const char * com_path)
{
	hComm = OpenPort(com_path);
	SetPortBaudRate(hComm, 115200);
	SetPortDataBits(hComm, 8);
	SetPortParity(hComm, 0);
	SetPortStopBits(hComm, 1);
	return 0;
}

int serial_close(void)
{
	ClosePort(hComm);
	return 0;
}

int serial_isopen(void)
{
	return (hComm != 0) ? 0: -1;
}

int serial_read(uint8_t* data, size_t size)
{
	int n = -1;
	if(serial_isopen() != -1)
	{
		do {
			n = ReceiveData(hComm, data, size);
		} while (n == -1);
	}
	return n;
}

int serial_write(const uint8_t* data, uint16_t size)
{
	int n = -1;
	if (serial_isopen() != -1)
	{
		n = SendData(hComm, data, size);
	}
	return n;
}



#endif



#ifdef LINUX

/**
@brief A generic pointer to the COM device (file descriptor)
*/
static int serial_port = -1;

#include <stdint.h>     /* C99 types */
#include <stdbool.h>    /* bool type */
#include <stdio.h>      /* printf fprintf */
#include <stdlib.h>     /* rand */
#include <unistd.h>     /* lseek, close */
#include <string.h>     /* memset */
#include <errno.h>      /* Error number definitions */
#include <termios.h>    /* POSIX terminal control definitions */
#include <fcntl.h>      /* Close */

static int set_interface_attribs_linux(int fd, int speed) {
    struct termios tty;

    memset(&tty, 0, sizeof tty);

    /* Get current attributes */
    if (tcgetattr(fd, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcgetattr failed with %d - %s", errno, strerror(errno));
        return -1;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    /* Control Modes */
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; /* set 8-bit characters */
    tty.c_cflag |= CLOCAL;                      /* local connection, no modem control */
    tty.c_cflag |= CREAD;                       /* enable receiving characters */
    tty.c_cflag &= ~PARENB;                     /* no parity */
    tty.c_cflag &= ~CSTOPB;                     /* one stop bit */
    /* Input Modes */
    tty.c_iflag &= ~IGNBRK;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    /* Output Modes */
    tty.c_oflag &= ~IGNBRK;
    tty.c_oflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    /* Local Modes */
    tty.c_lflag = 0;
    /* Settings for non-canonical mode */
    tty.c_cc[VMIN] = 0;                         /* non-blocking mode */
    tty.c_cc[VTIME] = 0;                        /* wait for (n * 0.1) seconds before returning */

    /* Set attributes */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcsetattr failed with %d - %s", errno, strerror(errno));
        return -1;
    }

    return 0;
}

/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

/* configure serial interface to be read blocking or not*/
static int set_blocking_linux(int fd, bool blocking) {
    struct termios tty;

    memset(&tty, 0, sizeof tty);

    /* Get current attributes */
    if (tcgetattr(fd, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcgetattr failed with %d - %s", errno, strerror(errno));
        return -1;
    }

    tty.c_cc[VMIN] = (blocking == true) ? 1 : 0;    /* set blocking or non-blocking mode */
    tty.c_cc[VTIME] = 1;                            /* wait for (n * 0.1) seconds before returning */

    /* Set attributes */
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        DEBUG_PRINTF("ERROR: tcsetattr failed with %d - %s", errno, strerror(errno));
        return -1;
    }

    return 0;
}



int serial_open(const char * com_path)
{
    /* Check input parameters */
    CHECK_NULL(com_path);

    char portname[50];
    int x;
    int fd;
    uint8_t data;
    ssize_t n;

    /* open tty port */
    sprintf(portname, "%s", com_path);
    fd = open(portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        printf("ERROR: failed to open COM port %s - %s\n", portname, strerror(errno));
    } else {
        printf("INFO: Configuring TTY\n");
        x = set_interface_attribs_linux(fd, B115200);
        if (x != 0) {
            printf("ERROR: failed to configure COM port %s\n", portname);
            return -1;
        }

        /* flush tty port before setting it as blocking */
        printf("INFO: Flushing TTY\n");
        do {
            n = read(fd, &data, 1);
            if (n > 0) {
                printf("NOTE: flushing serial port (0x%2X)\n", data);
            }
        } while (n > 0);

        /* set tty port blocking */
        printf("INFO: Setting TTY in blocking mode\n");
        x = set_blocking_linux(fd, true);
        if (x != 0) {
            printf("ERROR: failed to configure COM port %s\n", portname);
            return -1;
        }

        serial_port = fd;
	}
	return 0;

}

int serial_close(void)
{
	int x = -1;
	if(serial_port != -1)
	{
		x = close(serial_port);
		serial_port = -1;
	}
	return x;
}

int serial_read(uint8_t* data, size_t size)
{
	int n = -1;
	if(serial_port != -1)
	{
		do {
			n = read(serial_port, data, size);
		} while (n == -1 && errno == EINTR);
	}
	return n;
}

int serial_write(uint8_t* data, uint16_t size)
{
	int n = -1;
	if (serial_port != -1)
	{
		n = write(serial_port, data, size);
	}
	return n;
}

int serial_isopen(void)
{
	return (serial_port != -1) ? 0: -1;
}

#endif
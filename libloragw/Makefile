### get external defined data

LIBLORAGW_VERSION := "2.1.0"
include library.cfg
include ../target.cfg

### constant symbols

ARCH ?=
CROSS_COMPILE ?=
CC := $(CROSS_COMPILE)gcc
AR := $(CROSS_COMPILE)ar

CFLAGS := -O2 -Wall -Wextra -std=c99 -Iinc -I. -I../libtools/inc

OBJDIR = obj
INCLUDES = $(wildcard inc/*.h) $(wildcard ../libtools/inc/*.h)

### linking options

LIBS := -lloragw -ltinymt32 -lrt -lm

### general build targets

all: 	libloragw.a \
		test_loragw_com \
		test_loragw_reg \
		test_loragw_hal_tx \
		test_loragw_hal_rx \
		test_loragw_capture_ram \
		test_loragw_com_sx1250 \
		test_loragw_com_sx1261 \
		test_loragw_counter \
		test_loragw_toa \
		test_loragw_sx1261_rssi

clean:
	rm -f libloragw.a
	rm -f test_loragw_*
	rm -f $(OBJDIR)/*.o
	rm -f inc/config.h

install:
ifneq ($(strip $(TARGET_IP)),)
 ifneq ($(strip $(TARGET_DIR)),)
  ifneq ($(strip $(TARGET_USR)),)
	@echo "---- Copying libloragw files to $(TARGET_IP):$(TARGET_DIR)"
	@ssh $(TARGET_USR)@$(TARGET_IP) "mkdir -p $(TARGET_DIR)"
	@scp test_loragw_* $(TARGET_USR)@$(TARGET_IP):$(TARGET_DIR)
	@scp ../tools/reset_lgw.sh $(TARGET_USR)@$(TARGET_IP):$(TARGET_DIR)
  else
	@echo "ERROR: TARGET_USR is not configured in target.cfg"
  endif
 else
	@echo "ERROR: TARGET_DIR is not configured in target.cfg"
 endif
else
	@echo "ERROR: TARGET_IP is not configured in target.cfg"
endif

### transpose library.cfg into a C header file : config.h

inc/config.h: ../VERSION library.cfg
	@echo "*** Checking libloragw library configuration ***"
	@rm -f $@
	#File initialization
	@echo "#ifndef _LORAGW_CONFIGURATION_H" >> $@
	@echo "#define _LORAGW_CONFIGURATION_H" >> $@
	# Release version
	@echo "Release version   : $(LIBLORAGW_VERSION)"
	@echo "	#define LIBLORAGW_VERSION	"\"$(LIBLORAGW_VERSION)\""" >> $@
	# Debug options
	@echo "	#define DEBUG_AUX		$(DEBUG_AUX)" >> $@
	@echo "	#define DEBUG_COM		$(DEBUG_COM)" >> $@
	@echo "	#define DEBUG_MCU		$(DEBUG_MCU)" >> $@
	@echo "	#define DEBUG_I2C		$(DEBUG_I2C)" >> $@
	@echo "	#define DEBUG_REG		$(DEBUG_REG)" >> $@
	@echo "	#define DEBUG_HAL		$(DEBUG_HAL)" >> $@
	@echo "	#define DEBUG_GPS		$(DEBUG_GPS)" >> $@
	@echo "	#define DEBUG_GPIO		$(DEBUG_GPIO)" >> $@
	@echo "	#define DEBUG_LBT		$(DEBUG_LBT)" >> $@
	@echo "	#define DEBUG_RAD		$(DEBUG_RAD)" >> $@
	@echo "	#define DEBUG_CAL		$(DEBUG_CAL)" >> $@
	@echo "	#define DEBUG_SX1302	$(DEBUG_SX1302)" >> $@
	@echo "	#define DEBUG_FTIME		$(DEBUG_FTIME)" >> $@
	# end of file
	@echo "#endif" >> $@
	@echo "*** Configuration seems ok ***"

### library module target

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: src/%.c $(INCLUDES) inc/config.h | $(OBJDIR)
	$(CC) -c $(CFLAGS) $< -o $@

### static library

libloragw.a: $(OBJDIR)/loragw_com.o \
			 $(OBJDIR)/loragw_mcu.o \
			 $(OBJDIR)/sx1250_com.o \
			 $(OBJDIR)/sx1261_com.o \
			 $(OBJDIR)/loragw_aux.o \
			 $(OBJDIR)/loragw_reg.o \
			 $(OBJDIR)/loragw_sx1250.o \
			 $(OBJDIR)/loragw_sx1261.o \
			 $(OBJDIR)/loragw_sx1302.o \
			 $(OBJDIR)/loragw_hal.o \
			 $(OBJDIR)/loragw_lbt.o \
			 $(OBJDIR)/loragw_sx1302_timestamp.o \
			 $(OBJDIR)/loragw_sx1302_rx.o \
			 $(OBJDIR)/serial_port.o
	$(AR) rcs $@ $^

libloragw.dll: $(OBJDIR)/loragw_com.o \
			 $(OBJDIR)/loragw_mcu.o \
			 $(OBJDIR)/sx1250_com.o \
			 $(OBJDIR)/sx1261_com.o \
			 $(OBJDIR)/loragw_aux.o \
			 $(OBJDIR)/loragw_reg.o \
			 $(OBJDIR)/loragw_sx1250.o \
			 $(OBJDIR)/loragw_sx1261.o \
			 $(OBJDIR)/loragw_sx1302.o \
			 $(OBJDIR)/loragw_hal.o \
			 $(OBJDIR)/loragw_lbt.o \
			 $(OBJDIR)/loragw_sx1302_timestamp.o \
			 $(OBJDIR)/loragw_sx1302_rx.o \
			 $(OBJDIR)/serial_port.o
	$(CC) $(CFLAGS) -shared -o $@ $^

### test programs

test_loragw_com: tst/test_loragw_com.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools $< -o $@ $(LIBS)

test_loragw_reg: tst/test_loragw_reg.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools $< -o $@ $(LIBS)

test_loragw_hal_tx: tst/test_loragw_hal_tx.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools $< -o $@ $(LIBS)

test_loragw_hal_rx: tst/test_loragw_hal_rx.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools $< -o $@ $(LIBS)

test_loragw_capture_ram: tst/test_loragw_capture_ram.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools  $< -o $@ $(LIBS)

test_loragw_com_sx1250: tst/test_loragw_com_sx1250.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools  $< -o $@ $(LIBS)

test_loragw_com_sx1261: tst/test_loragw_com_sx1261.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools  $< -o $@ $(LIBS)

test_loragw_counter: tst/test_loragw_counter.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools  $< -o $@ $(LIBS)

test_loragw_gps: tst/test_loragw_gps.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools  $< -o $@ $(LIBS)

test_loragw_toa: tst/test_loragw_toa.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools  $< -o $@ $(LIBS)

test_loragw_sx1261_rssi: tst/test_loragw_sx1261_rssi.c libloragw.a
	$(CC) $(CFLAGS) -L. -L../libtools  $< -o $@ $(LIBS)

### EOF

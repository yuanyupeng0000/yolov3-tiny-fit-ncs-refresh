# Copyright (C) 2018 Intel Corporation
#
# SPDX-License-Identifier: Apache-2.0

INSTALLDIR := ${DESTDIR}/usr/local
LIBUSB_CFLAGS := $(shell pkg-config --cflags)
LDFLAGS += $(shell pkg-config --libs) -ldl -ljson-c -lpthread -ludev
LDFLAGS += -ldl -z noexecstack -z relro -z now

CFLAGS += -O2 -Wall $(LIBUSB_CFLAGS) -I../include -fPIC -D_FORTIFY_SOURCE=2
CFLAGS += -fPIC -MMD -MP -Wformat -Wformat-security -fstack-protector-strong
#CFLAGS += -DDEBUG

OBJS = i2cbusses.o \
			 smbus_linux.o \
			 mcu.o \
			 ioexpander.o \
			 hidapi_linux.o \
			 hid_f75114.o \
			 main.o \
			 thread_linux.o \
			 osl_linux.c \
			 bsl_error_code.c \
			 bsl_cfg.o

ifeq "$(wildcard $(INSTALLDIR) )" " "
	INSTALLDIR = ${DESTDIR}/usr
endif

all: libbsl.so.0 tool
tool: bsl_reset

*.o: i2c-dev.h hid-dev.h ../include/hddl-bsl.h i2cbusses.h

.c.o:
	$(CC) -c  $(CFLAGS) $<

libbsl.so.0: $(OBJS)
	$(CC) -shared -o $@ -Wl,-h$@ $(CFLAGS)  $(OBJS) $(LDFLAGS)
	ln -fs libbsl.so.0 libbsl.so

clean:
	rm -f *.o *.d libbsl.so.0 libbsl.so bsl_reset

bsl_reset: bsl_reset.o
	$(CC) $(LDFLAGS) -o $@ bsl_reset.o  -L./ -lbsl

basicinstall: libbsl.so.0
	mkdir -p $(INSTALLDIR)/include/
	mkdir -p $(INSTALLDIR)/lib/
	mkdir -p $(INSTALLDIR)/bin/
	cp libbsl.so.0 $(INSTALLDIR)/lib/
	ln -fs libbsl.so.0 $(INSTALLDIR)/lib/libbsl.so
	cp ../include/* $(INSTALLDIR)/include/
	cp bsl_reset $(INSTALLDIR)/bin/
	bash generate_udev_rules.sh ${DESTDIR}/etc/udev/rules.d/

postinstall:
	modprobe i2c-i801
	udevadm control --reload-rules
	udevadm trigger
	ldconfig

install: basicinstall postinstall

uninstall:
	rm $(INSTALLDIR)/lib/libbsl.so.0
	rm $(INSTALLDIR)/lib/libbsl.so
	rm $(INSTALLDIR)/include/hddl-bsl.h

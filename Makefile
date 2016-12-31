#### This file is part of the avr-gcc-examples project.
##
## Copyright (C) 2008 Uwe Hermann <uwe@hermann-uwe.de>
##
## This program is free software; you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation; either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program; if not, write to the Free Software
## Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
##

PROGRAM = WDTTicker
MCU = attiny13
#MCU = attiny85
CC  = /opt/local/bin/avr-gcc
OBJCOPY = /opt/local/bin/avr-objcopy
CFLAGS += -Wall -g -Os -mmcu=$(MCU)
#CFLAGS += -Wall -g3 -gdwarf-2 -Os -mmcu=$(MCU)
LDFLAGS +=
OBJS = $(PROGRAM).o

# ATTiny13
FUSE_LOW  = 0x75
FUSE_HIGH = 0xFF
# ATTiny85
#FUSE_LOW  = 0xE3
#FUSE_HIGH = 0xDF

# Be silent per default, but 'make V=1' will show all compiler calls.
# ifneq ($(V),1)
# Q := @
# endif

all: $(PROGRAM).hex size

$(PROGRAM).elf: $(PROGRAM).o
	@printf "  LD      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

$(PROGRAM).hex: $(PROGRAM).elf
	@printf "  OBJCOPY $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(OBJCOPY) -O ihex $< $@

%.o: %.cpp
	@printf "  CC      $(subst $(shell pwd)/,,$(@))\n"
	$(Q)$(CC) $(CFLAGS) -o $@ -c $<

size: $(PROGRAM).elf
#	$(Q)/opt/local/bin/avr-size -C --mcu=$(MCU) $(PROGRAM).elf
	/Users/danny/bin/avr-size -C --mcu=$(MCU) $(PROGRAM).elf

flash: $(PROGRAM).hex
	@printf "  FLASH   $(PROGRAM).hex\n"
#	$(Q)avrdude -c avrispv2 -P usb -p t13 -U flash:w:$(PROGRAM).hex
#	$(Q)avrdude -p t13 -c usbasp -u -U flash:w:$(PROGRAM).hex:a -U hfuse:w:$(FUSE_HIGH):m  -U lfuse:w:$(FUSE_LOW):m 
#	$(Q)avrdude -p t13 -c usbasp -u -U flash:w:$(PROGRAM).hex:a
#	$(Q)avrdude -p t85 -c usbasp -u -U flash:w:$(PROGRAM).hex:a
	/opt/local/bin/avrdude -p t13 -c usbasp -u -U flash:w:$(PROGRAM).hex:a

fuse: 
	@printf "  FLASH   $(PROGRAM).hex\n"
#	$(Q)avrdude -c avrispv2 -P usb -p t13 -U flash:w:$(PROGRAM).hex
#	$(Q)avrdude -p t13 -c usbasp -u -U hfuse:w:$(FUSE_HIGH):m  -U lfuse:w:$(FUSE_LOW):m
#	$(Q)avrdude -p t85 -c usbasp -u -U hfuse:w:$(FUSE_HIGH):m  -U lfuse:w:$(FUSE_LOW):m 
	/opt/local/bin/avrdude -p t13 -c usbasp -u -U hfuse:w:$(FUSE_HIGH):m  -U lfuse:w:$(FUSE_LOW):m


clean:
	@printf "  CLEAN   $(subst $(shell pwd)/,,$(OBJS))\n"
	$(Q)rm -f $(OBJS)
	@printf "  CLEAN   $(PROGRAM).elf\n"
	$(Q)rm -f *.elf
	@printf "  CLEAN   $(PROGRAM).hex\n"
	$(Q)rm -f *.hex


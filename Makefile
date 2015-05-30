####################################
# stm32 timer led example Makefile
####################################

PROGRAM = timer_led

ENTRY_FILE = startup

ENTRY_CFILE = $(ENTRY_FILE).c
ENTRY_OFILE = $(ENTRY_FILE).o

ALL_OFILES = main.o $(ENTRY_OFILE)

TARGET_BIN = $(PROGRAM).bin
TARGET_ELF = $(PROGRAM).elf

CC = arm-none-eabi-gcc
LD = arm-none-eabi-ld
CP = arm-none-eabi-objcopy

LKR_SCRIPT = $(PROGRAM).ld

FAMILY = STM32F10X_MD

DEFINES = -D$(FAMILY)
INCLUDES = -I./Libraries/CMSIS/CM3/DeviceSupport/ST/STM32F10x/ \
           -I./Libraries/CMSIS/CM3/CoreSupport/ \
           -I./Libraries/STM32F10x_StdPeriph_Driver/inc

CFLAGS  = -c -march=armv7-m -mcpu=cortex-m3 -mthumb \
          -fno-common -nostdlib -fno-builtin -ffreestanding \
          -Wall -O0 -g  \
          $(DEFINES)

LFLAGS  = -nostartfiles -T$(LKR_SCRIPT)
CPFLAGS = -Obinary

.PHONY: all clean write

all: $(TARGET_BIN)

$(TARGET_BIN): $(TARGET_ELF)
	$(CP) $(CPFLAGS) $< $@

$(TARGET_ELF): $(ALL_OFILES)
	$(LD) $(LFLAGS) $(ALL_OFILES) -o $@ 

main.o: main.c
	$(CC) $(INCLUDES) $(CFLAGS) main.c -o $@

$(ENTRY_OFILE): $(ENTRY_CFILE)
	$(CC) $(INCLUDES) $(CFLAGS) $(ENTRY_CFILE) -o $@

clean:
	rm -rf *.o *.elf *.bin

write: 
	./write_bin.sh openocd.cfg $(TARGET_ELF)

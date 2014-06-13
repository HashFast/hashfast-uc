.PHONY:	all program clean

DEBUG = 1

OUTPUT = uc3_loader

all:	$(OUTPUT).elf $(OUTPUT).hex

CHIP = uc3b0512
JTAG_CHIP = at32$(CHIP)
JTAG_POD = jtagice3

ifeq "$(OS)" "Windows_NT"
ATMELSTUDIO_DIR = "/cygdrive/c/Program Files (x86)/Atmel/Atmel Studio 6.1"
AVR32_TOOLCHAIN_DIR = "/cygdrive/c/Program Files (x86)/Atmel/AVR Tools/AVR Toolchain"
AVR32_HEADERS_DIR = $(wildcard ~/avrtools/atmel-headers-6.1.3.1475)
else
AVR32_TOOLCHAIN_DIR = $(wildcard ~/avrtools/avr32-gnu-toolchain-linux_x86_64)
AVR32_HEADERS_DIR = $(wildcard ~/avrtools/atmel-headers-6.1.3.1475)
endif


AVR32_TOOLCHAIN_BIN_DIR = $(AVR32_TOOLCHAIN_DIR)/bin
AS = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-as
CC = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-gcc
LD = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-ld
OBJCOPY = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-objcopy
OBJDUMP = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-objdump
SIZE = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-size
ATPROGRAM = $(ATMELSTUDIO_DIR)/atbackend/atprogram

MKDIR_P = mkdir -p

S = ../src
S_APP = ../../uc3/src

LDSCRIPT = $(S)/ASF_modified/link_$(CHIP).lds

INCLUDE_DIRS = \
	$(S) \
	$(S)/ASF_modified \
	$(S)/ASF \
	$(AVR32_HEADERS_DIR) \
	$(S_APP) \

CFLAGS = 

CCOPTS = $(CFLAGS)
CCOPTS += $(foreach INC,$(INCLUDE_DIRS),-I$(INC))
ifeq ($(DEBUG), 1)
CCOPTS += -O0 -g3
else
CCOPTS += -Os -mrelax
endif
CCOPTS += \
	-c \
	-fdata-sections -ffunction-sections -masm-addr-pseudos \
	-Wall \
	-mpart=$(CHIP) \
	-std=gnu99 -fno-strict-aliasing \
	-Wstrict-prototypes -Wmissing-prototypes \
	-Werror-implicit-function-declaration -Wpointer-arith \
	-mno-cond-exec-before-reload \
	-MD -MP

ASOPTS = \
	-D__ASSEMBLY__
ASOPTS += $(foreach INC,$(INCLUDE_DIRS),-Wa,-I$(INC))
ASOPTS += $(foreach INC,$(INCLUDE_DIRS),-I$(INC))
ifeq ($(DEBUG), 0)
ASOPTS += -mrelax
endif
ASOPTS += \
	-mpart=$(CHIP) \
	-c \
	-x assembler-with-cpp \
	-MD -MP


LDOPTS = \
	-nostartfiles \
	-Wl,-Map=$*.map \
	-Wl,-start-group -lm -Wl,--end-group \
	-Wl,--gc-sections --rodata-writable -Wl,--direct-data \
	-mpart=$(CHIP) \
	-T$(LDSCRIPT) \
	-Wl,-e,_start
ifeq ($(DEBUG), 0)
LDOPTS += -Wl,--relax
endif

C_SOURCES = \
	ASF/udc.c \
	ASF/udi_vendor.c \
	ASF/udi_vendor_desc.c \
	ASF_modified/usbb_device.c \
	chain.c \
	twi.c \
	usbdev.c \
	twicomms.c \
	flash.c \
	utils.c \
	gpio.c \
	timers.c \
	interrupts.c \
	crc.c \
	main.c

S_SOURCES = \
	ASF/exception.S \
	ASF/startup_uc3.S

C_OBJS = $(C_SOURCES:%.c=%.o)

C_DEPS = $(C_SOURCES:%.c=%.d)

S_OBJS = $(S_SOURCES:%.S=%.o)

S_DEPS = $(S_SOURCES:%.S=%.d)

OBJS = $(C_OBJS) $(S_OBJS)

DEPS = $(C_DEPS) $(S_DEPS)

%.hex:	%.elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature $< $@

%.elf:	$(OBJS) $(LDSCRIPT)
	$(CC) -o$@ $(OBJS) $(LDOPTS)
	$(SIZE) $@


$(C_OBJS):	%.o: $(S)/%.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CCOPTS) -o $@ $<


$(S_OBJS):	%.o: $(S)/%.S
	$(MKDIR_P) $(dir $@)
	$(CC) $(ASOPTS) -o $@ $<


# I have not done a cygwin build of the avr32 toolchain; I am using the
# binaries distributed by Atmel instead.  Those binaries were not built
# for cygwin (although they do deal with cygwin paths for the source files
# and include paths).  One of the consequences is that they use windows
# style paths instead of cygwin's unix style paths when analyzing
# dependencies.  That screws up the .d files so that cygwin make cannot
# deal with them.  So dependencies are broken under Windows.  This project
# is small enough that I just do a "make clean" followed by "make" to
# ensure everything is rebuilt after changing any header files.  If someone
# has a little time they might want to dig into ways to deal with this that
# would allow auto generated dependencies to work here.  Building the
# avr32-gcc toolchain for cygwin would probably do it, but there may be
# workarounds that would not require that.
ifneq "$(OS)" "Windows_NT"
-include $(DEPS)
endif


program:	$(OUTPUT).hex
	$(ATPROGRAM) -t $(JTAG_POD) -i jtag -d $(JTAG_CHIP) chiperase
	$(ATPROGRAM) -t $(JTAG_POD) -i jtag -d $(JTAG_CHIP) \
	             program -f $< --format ihex --verify \
	             write -fs -o 0xfffe1410 --values 0xfff3ffff --verify

clean:
	-$(RM) $(OBJS) $(DEPS)
	-$(RM) $(OUTPUT).elf $(OUTPUT).map $(OUTPUT).hex 
	-$(RM) -r ASF ASF_modified


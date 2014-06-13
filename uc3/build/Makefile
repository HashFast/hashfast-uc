.PHONY:	all program programdfu programhf programsierra enterloader clean

DEBUG = 0

#sudo apt-get install srecord
OUTPUT = uc3

# which module in chain to program with programhf target
HF_TARGET_MODULE = 0

FLASH_SIZE = 512
CHIP = uc3b0$(FLASH_SIZE)
DFU_CHIP = at32$(CHIP)
LDSCRIPT = $(S)/ASF/avr32/utils/linker_scripts/at32uc3b/0$(FLASH_SIZE)/gcc/link_$(CHIP).lds

ifeq "$(OS)" "Windows_NT"
ATMELSTUDIO_DIR = "/cygdrive/c/Program Files (x86)/Atmel/Atmel Studio 6.1"
AVR32_TOOLCHAIN_DIR = "/cygdrive/c/Program Files (x86)/Atmel/AVR Tools/AVR Toolchain"
AVR32_HEADERS_DIR = $(wildcard ~/avrtools/atmel-headers-6.1.3.1475)
else
AVR32_TOOLCHAIN_DIR = $(wildcard ~/avrtools/avr32-gnu-toolchain-linux_x86_64)
AVR32_HEADERS_DIR = $(wildcard ~/avrtools/atmel-headers-6.1.3.1475)
endif
ifndef HF_TOOLS_DIR
HF_TOOLS_DIR = ../../../tools
endif

AVR32_TOOLCHAIN_BIN_DIR = $(AVR32_TOOLCHAIN_DIR)/bin
AS = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-as
CC = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-gcc
LD = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-ld
OBJCOPY = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-objcopy
OBJDUMP = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-objdump
SIZE = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-size
CREATEUPDATE = $(HF_TOOLS_DIR)/utils/createupdate
HFUPDATE = $(HF_TOOLS_DIR)/utils/hfupdate
ENTERLOADER = $(HF_TOOLS_DIR)/utils/enterloader
DFU = dfu-programmer
SREC_CAT = srec_cat
MKDIR_P = mkdir -p


all:	$(OUTPUT).elf $(OUTPUT).hex $(OUTPUT).cropped.hex

ifneq  ($(wildcard $(CREATEUPDATE)),)
all:    $(OUTPUT).cropped.hfu
endif

S = ../src
SL = ../../uc3_loader/src

INCLUDE_DIRS = \
	$(S) \
	$(S)/config \
	$(S)/ASF/avr32/utils \
	$(S)/ASF/common/services/usb/class/cdc/device \
	$(S)/ASF/common/services/usb/udc \
	$(S)/ASF/common/services/usb/cdc \
	$(S)/ASF/avr32/boards/evk1101 \
	$(S)/ASF/common/utils \
	$(S)/ASF/common/services/sleepmgr \
	$(S)/ASF/common/services/clock \
	$(S)/ASF/avr32/drivers/usbb \
	$(S)/ASF/avr32/boards \
	$(S)/ASF/avr32/drivers/gpio \
	$(S)/ASF/common/services/usb/class/cdc \
	$(S)/ASF/common/boards \
	$(S)/ASF/avr32/utils/preprocessor \
	$(S)/ASF/avr32/drivers/intc \
	$(S)/ASF/avr32/drivers/usart \
	$(S)/ASF/avr32/drivers/pm \
	$(S)/ASF/common/services/usb \
	$(S)/ASF/avr32/drivers/flashc \
	$(S)/ASF/avr32/drivers/pdca \
	$(S)/ASF/avr32/drivers/rtc \
	$(S)/ASF/avr32/drivers/spi \
	$(S)/ASF/avr32/drivers/tc \
	$(S)/ASF/avr32/drivers/pwm \
	$(S)/ASF/avr32/drivers/wdt \
	$(S)/ASF/common/drivers/nvm \
	$(S)/ASF/common/services/spi/uc3_spi \
	$(S)/ASF/common/services/spi \
	$(S)/ASF/common/services/ioport \
	$(SL) \
	$(AVR32_HEADERS_DIR)

CFLAGS = \
	-DBOARD=EVK1101 \
	-DUDD_ENABLE

CCOPTS = $(CFLAGS)
CCOPTS += $(foreach INC,$(INCLUDE_DIRS),-I$(INC))
ifeq ($(DEBUG), 1)
CCOPTS += -DDEBUG -O0 -g3
else
CCOPTS += -DNDEBUG -O2 -mrelax
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
	-Wl,-e,_trampoline -Wl,--check-sections
ifeq ($(DEBUG), 0)
LDOPTS += -Wl,--relax
endif


C_SOURCES = \
	boardid.c \
	fan_control.c \
	hf_nvram.c \
	hf_pid.c \
	module_handler.c \
	pll_calc.c \
	spi_handler.c \
	twi.c \
	twi_handler.c \
	usbctrl.c \
	da2s.c \
	adc.c \
	watchdog.c \
	cores.c \
	asic_handler.c \
	gwq_handler.c \
	hf_trace.c \
	uprintf.c \
	usb_uart.c \
	mpu.c \
	profile.c \
	cli.c \
	tty.c \
	trap.c \
	ASF/avr32/drivers/pdca/pdca.c \
	ASF/avr32/drivers/pwm/pwm.c \
	ASF/avr32/drivers/rtc/rtc.c \
	ASF/avr32/drivers/spi/spi.c \
	ASF/avr32/drivers/tc/tc.c \
	ASF/avr32/drivers/wdt/wdt.c \
	ASF/common/drivers/nvm/uc3/uc3_nvm.c \
	ASF/common/services/spi/uc3_spi/spi_master.c \
	ASF/common/services/usb/class/cdc/device/example/at32uc3b0256_evk1101/ui.c \
	ASF/avr32/boards/evk1101/init.c \
	ASF/avr32/boards/evk1101/led.c \
	ASF/avr32/drivers/flashc/flashc.c \
	ASF/avr32/drivers/gpio/gpio.c \
	ASF/avr32/drivers/intc/intc.c \
	ASF/avr32/drivers/pm/pm.c \
	ASF/avr32/drivers/pm/pm_conf_clocks.c \
	ASF/avr32/drivers/pm/power_clocks_lib.c \
	ASF/avr32/drivers/usart/usart.c \
	ASF/avr32/drivers/usbb/usbb_device.c \
	ASF/common/services/clock/uc3b0_b1/sysclk.c \
	ASF/common/services/sleepmgr/uc3/sleepmgr.c \
	ASF/common/services/usb/class/cdc/device/udi_cdc.c \
	ASF/common/services/usb/class/cdc/device/udi_cdc_desc.c \
	ASF/common/services/usb/udc/udc.c \
	hf_util.c \
	main.c

S_SOURCES = \
	ASF/avr32/utils/startup/trampoline_uc3.S \
	ASF/avr32/drivers/intc/exception.S \
	ASF/avr32/utils/startup/startup_uc3.S

C_OBJS = $(C_SOURCES:%.c=%.o)

C_DEPS = $(C_SOURCES:%.c=%.d)

S_OBJS = $(S_SOURCES:%.S=%.o)

S_DEPS = $(S_SOURCES:%.S=%.d)

OBJS = $(C_OBJS) $(S_OBJS)

DEPS = $(C_DEPS) $(S_DEPS)

%.hfu:	%.bin
	$(CREATEUPDATE) -f$(FLASH_SIZE) $< $@

%.bin:	%.hex
	# exclude trampoline and create binary image
	$(SREC_CAT) $< -intel -exclude 0x80000000 0x80008000 \
		-offset -0x80008000 -o $@ -binary

%.cropped.hex:	%.hex
	# exclude user page
	$(SREC_CAT) $< -intel -exclude 0x80800000 0x80800200 -o $@ -intel

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
# deal with them.  So dependencies are broken under Windows.  A crude
# workaround is to manually do a "make clean" after changing any header
# files to ensure everything is rebuilt.  If someone has a little time
# they might want to dig into ways to deal with this that would allow
# auto generated dependencies to work here.  Building the avr32-gcc
# toolchain for cygwin would probably do it, but there may be workarounds
# that would not require that.
ifneq "$(OS)" "Windows_NT"
-include $(DEPS)
endif

# alias
program:	programhf

programdfu:	$(OUTPUT).cropped.hex
	$(DFU) $(DFU_CHIP) erase
	$(DFU) $(DFU_CHIP) flash --suppress-bootloader-mem $<
	# dfu-programmer is returning 1 for reset commands even when they work
	-$(DFU) $(DFU_CHIP) reset

programhf:	$(OUTPUT).cropped.hfu
	$(HFUPDATE) -m$(HF_TARGET_MODULE) $< -r

programsierra:	$(OUTPUT).cropped.hfu
	$(HFUPDATE) -m0 $<
	$(HFUPDATE) -m1 $<
	$(HFUPDATE) -m2 $<
	$(HFUPDATE) -r

enterloader:
        # Ignore error if already in HFU mode
	-$(ENTERLOADER)
	sleep 3
	$(HFUPDATE) -E

clean:
	-$(RM) $(OBJS) $(DEPS)
	-$(RM) $(OUTPUT).elf $(OUTPUT).map $(OUTPUT).hex $(OUTPUT).cropped.hex
	-$(RM) $(OUTPUT).cropped.bin $(OUTPUT).cropped.hfu
	-$(RM) -r ASF


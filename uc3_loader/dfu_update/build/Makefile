.PHONY:	all program programdfu programhf clean

OUTPUT = uc3_loader_dfu_update.$(FLASH_SIZE)k

# which module in chain to program with programhf target
HF_TARGET_MODULE = 0

CHIP = uc3b0$(FLASH_SIZE)
DFU_CHIP = at32$(CHIP)

ifeq "$(OS)" "Windows_NT"
ATMELSTUDIO_DIR = "/cygdrive/c/Program Files (x86)/Atmel/Atmel Studio 6.1"
AVR32_TOOLCHAIN_DIR = "/cygdrive/c/Program Files (x86)/Atmel/AVR Tools/AVR Toolchain"
AVR32_HEADERS_DIR = $(wildcard ~/avrtools/atmel-headers-6.1.3.1475)
else
AVR32_TOOLCHAIN_DIR = $(wildcard ~/avrtools/avr32-gnu-toolchain-linux_x86_64)
AVR32_HEADERS_DIR = $(wildcard ~/avrtools/atmel-headers-6.1.3.1475)
endif
ifndef HF_TOOLS_DIR
HF_TOOLS_DIR = ../../../../tools
endif

AVR32_TOOLCHAIN_BIN_DIR = $(AVR32_TOOLCHAIN_DIR)/bin
AS = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-as
CC = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-gcc
LD = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-ld
OBJCOPY = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-objcopy
OBJDUMP = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-objdump
SIZE = $(AVR32_TOOLCHAIN_BIN_DIR)/avr32-size
CREATEUPDATE = $(HF_TOOLS_DIR)/createupdate
HFUPDATE = $(HF_TOOLS_DIR)/hfupdate
SREC_CAT = srec_cat
DFU = dfu-programmer
MKDIR_P = mkdir -p

ifeq ($(FLASH_SIZE),)

all:
	make FLASH_SIZE=512
	make FLASH_SIZE=256
	make FLASH_SIZE=128 
else

all:    $(OUTPUT).elf $(OUTPUT).hex
ifneq  ($(wildcard $(CREATEUPDATE)),)
all:	  $(OUTPUT).hfu
endif

endif

S = ../src

LDSCRIPT = $(S)/ASF_modified/link_$(CHIP).lds

INCLUDE_DIRS = \
	$(S) \
	./src/$(FLASH_SIZE)k \
	$(S)/ASF_modified \
	$(S)/ASF \
	$(AVR32_HEADERS_DIR)

CFLAGS = 

CCOPTS = $(CFLAGS)
CCOPTS += $(foreach INC,$(INCLUDE_DIRS),-I$(INC))
CCOPTS += \
	-c \
	-O0 -g3 \
	-fdata-sections -ffunction-sections -masm-addr-pseudos \
	-Wall \
	-mpart=$(CHIP) \
	-std=gnu99 -fno-strict-aliasing \
	-Wstrict-prototypes -Wmissing-prototypes \
	-Werror-implicit-function-declaration -Wpointer-arith \
	-mrelax -mno-cond-exec-before-reload \
	-MD -MP

ASOPTS = \
	-D__ASSEMBLY__
ASOPTS += $(foreach INC,$(INCLUDE_DIRS),-Wa,-I$(INC))
ASOPTS += $(foreach INC,$(INCLUDE_DIRS),-I$(INC))
ASOPTS += \
	-mpart=$(CHIP) \
	-c \
	-mrelax \
	-x assembler-with-cpp \
	-MD -MP


C_SOURCES = \
	flash.c \
	utils.c \
	interrupts.c \
	../build/src/$(FLASH_SIZE)k/update_data.c \
	main.c

S_SOURCES = \
	ASF_modified/trampoline_uc3.S \
	ASF/exception.S \
	ASF/startup_uc3.S

C_OBJS  = $(C_SOURCES:%.c=%.o)
_C_OBJS = $(patsubst %.o,obj/$(FLASH_SIZE)k/%.o,$(C_OBJS))

C_DEPS  = $(C_SOURCES:%.c=%.d)
_C_DEPS = $(patsubst %.d,obj/$(FLASH_SIZE)k/%.d,$(C_DEPS))

S_OBJS  = $(S_SOURCES:%.S=%.o)
_S_OBJS = $(patsubst %.o,obj/$(FLASH_SIZE)k/%.o,$(S_OBJS))

S_DEPS  = $(S_SOURCES:%.S=%.d)
_S_DEPS = $(patsubst %.d,obj/$(FLASH_SIZE)k/%.d,$(S_DEPS))

OBJS    = $(_C_OBJS) $(_S_OBJS)

DEPS    = $(_C_DEPS) $(_S_DEPS)

../../build/uc3_loader.$(FLASH_SIZE)k.hex:
	make -C ../../build

$(S)/../build/src/$(FLASH_SIZE)k/update_data.c:	../../build/uc3_loader.$(FLASH_SIZE)k.hex
	$(MKDIR_P) $(S)/../build/src/$(FLASH_SIZE)k/
	$(SREC_CAT) $< -intel -o $@ -c-array update_data -include

%.hfu:	%.bin
	$(CREATEUPDATE) -f$(FLASH_SIZE) $< $@

%.bin:	%.hex
	# exclude trampoline and create binary image
	$(SREC_CAT) $< -intel -exclude 0x80000000 0x80008000 \
		-offset -0x80008000 -o $@ -binary

%.hex:	%.elf
	$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock -R .signature $< $@

%.elf:	$(OBJS) $(LDSCRIPT)
	$(CC) -o$@ $(OBJS) -nostartfiles \
		-Wl,-Map=$*.map \
		-Wl,-start-group -lm -Wl,--end-group \
		-Wl,--gc-sections --rodata-writable -Wl,--direct-data \
		-mpart=$(CHIP) \
		-Wl,--relax \
		-T$(LDSCRIPT) \
		-Wl,-e,_trampoline -Wl,--defsym=spurious=0x181c
	$(SIZE) $@


$(_C_OBJS):	obj/$(FLASH_SIZE)k/%.o: $(S)/%.c
	$(MKDIR_P) $(dir $@)
	$(CC) $(CCOPTS) -o $@ $<


$(_S_OBJS):	obj/$(FLASH_SIZE)k/%.o: $(S)/%.S
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

# alias
program:	programdfu

ifeq ($(FLASH_SIZE),)

programdfu:
	-make FLASH_SIZE=512 programdfu
	-make FLASH_SIZE=256 programdfu
	-make FLASH_SIZE=128 programdfu

programhf:
	# MUST SPECIFY FLASH_SIZE #

clean:
	make FLASH_SIZE=512 clean
	make FLASH_SIZE=256 clean
	make FLASH_SIZE=128 clean
	-$(RM) -r obj src

else

programdfu:	$(OUTPUT).hex
	$(DFU) $(DFU_CHIP) dump > /dev/null
	$(DFU) $(DFU_CHIP) erase
	$(DFU) $(DFU_CHIP) flash --suppress-bootloader-mem $<
	# dfu-programmer is returning 1 for reset commands even when they work
	-$(DFU) $(DFU_CHIP) reset

programhf:	$(OUTPUT).hfu
	$(HFUPDATE) -m$(HF_TARGET_MODULE) $< -r

clean:
	-$(RM) $(OBJS) $(DEPS)
	-$(RM) -r src/$(FLASH_SIZE)k
	-$(RM) $(OUTPUT).elf $(OUTPUT).map $(OUTPUT).hex $(OUTPUT).hfu
	-$(RM) -r obj/$(FLASH_SIZE)k/ASF obj/$(FLASH_SIZE)k/ASF_modified

endif

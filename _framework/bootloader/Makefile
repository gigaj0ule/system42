######################################################################
# STM32F103 Bootloader Makefile
# (c) Adam Munich 2020, All Rights Reserved

# This is part of an STM32 Framework organized by Adam Munich for 
# hobby projects. It is a work in progress. 
# It is for non-commercial use only, unless explicit permission 
# and terms of use are given.

# The name of your project (used to name the compiled .hex file)
TARGET = "fw"

# Family
ifndef MCU_FAMILY
	MCU_FAMILY := f103c8
endif

# Configurable options
OPTIONS := -DVERSION=5

ifeq ($(MCU_FAMILY), f103c8)
	OPTIONS += -DSTM32F1
	COREFILES_FAMILY := f1

else ifeq ($(MCU_FAMILY), f103rc)
	OPTIONS += -DSTM32F1
	COREFILES_FAMILY := f1

else ifeq ($(MCU_FAMILY), f40x)
	OPTIONS += -DSTM32F4
	COREFILES_FAMILY := f4
endif

ifeq ($(META_V5), 1)
	OPTIONS += -DMTV
endif

# Strings
OPTIONS += -DI_MANUFACTURER="\"Civil Electric\""
OPTIONS += -DI_PRODUCT="\"Bootloader\""
OPTIONS += -DID_VENDOR=0x$(USB_VENDOR_ID)
OPTIONS += -DID_PRODUCT=0x$(USB_PRODUCT_ID)

# Should SWD be enabled?
ifeq ($(USE_RDP), 1)
	OPTIONS += -DENABLE_READOUT_PROTECTION_LEVEL_1
endif

# Directory to build in
BUILDDIR = $(abspath $(CURDIR)/__build)


######################################################################
# Ecryption key for the particular instance of this bootloader
ifdef USE_BOOTLOADER_KEY_FILE
	OPTIONS += -DBOOT_KEY_FILE="\"$(USE_BOOTLOADER_KEY_FILE)\""
endif

######################################################################
# Location of utilities, libraries.

# Path of project source
BOOTLOADER_PATH = src/bootloader
OPENCM3PATH = src/lib
OPENCM3STM = $(OPENCM3PATH)/stm32
OPENCM3USB = $(OPENCM3PATH)/usb
OPENCM3BRANCH = $(OPENCM3STM)/$(CM3BRANCH)
OPENCM3COMMON = $(OPENCM3STM)/common

# Compiler options for C and C++
CPPFLAGS = $(OPTIONS)
CPPFLAGS += -mthumb -mthumb-interwork
CPPFLAGS += -Os -g  
CPPFLAGS += -fno-exceptions 
CPPFLAGS += -ffunction-sections 
CPPFLAGS += -fdata-sections
CPPFLAGS += -Wall -Wdouble-promotion -Wfloat-conversion -Wimplicit -Wunused-function
CPPFLAGS += -Wcast-align -Wpointer-arith -Wswitch -Wredundant-decls -Wreturn-type 
CPPFLAGS += -Wshadow -Wunused

ifeq ($(COREFILES_FAMILY), f1)
	CPPFLAGS += -mcpu=cortex-m3

else ifeq ($(COREFILES_FAMILY), f4)
	CPPFLAGS += -mcpu=cortex-m4 

endif


# Compiler options for C only
CFLAGS = -std=c11


# Linker options
LDFLAGS = -nostartfiles
LDFLAGS += -lc -lgcc
#LDFLAGS = -mcpu=cortex-m3 -mthumb -g --specs=nano.specs -u _printf_float
LDFLAGS += -Wl,--cref 
#LDFLAGS += -Wl,--check-sections
#LDFLAGS += -Wl,--defsym=LD_FLASH_OFFSET=0
#LDFLAGS += -Wl,--defsym=LD_MAX_SIZE=65536
#LDFLAGS += -Wl,--defsym=LD_MAX_DATA_SIZE=20480
#LDFLAGS += -Wl,--entry=Reset_Handler 
#LDFLAGS += -Wl,--unresolved-symbols=report-all 
LDFLAGS += -Wl,--gc-sections
#LDFLAGS += -Wl,--warn-common
LDFLAGS += -Wl,-Map,$(BUILDDIR)/$(TARGET).map
#LDFLAGS += -Wl,--start-group -Wl,--no-whole-archive -Wl,--end-group
#LDFLAGS += -lm -lc -lnosys -Wl,--library=arm_cortexM3l_math


######################################################################
# Environment tools

# Names for the compiler programs
CC = arm-none-eabi-gcc
CXX = arm-none-eabi-g++
AS = arm-none-eabi-as
AR = arm-none-eabi-gcc-ar
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
DUMP = arm-none-eabi-objdump
STRIP = arm-none-eabi-strip


######################################################################
# Automatically create lists of the sources and objects
rwildcard = $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

# Source
C_FILES := $(call rwildcard, $(BOOTLOADER_PATH), *.c)
C_FILES += $(call rwildcard, $(OPENCM3COMMON), *_all.c)
C_FILES += $(call rwildcard, $(OPENCM3COMMON), *_f.c)
C_FILES += $(OPENCM3STM)/desig.c

ifeq ($(COREFILES_FAMILY), f1)
	C_FILES += $(wildcard $(OPENCM3STM)/f1/*.c)
	C_FILES += $(wildcard $(OPENCM3COMMON)/*_f01.c)
	C_FILES += $(OPENCM3COMMON)/st_usbfs_core.c
	C_FILES += $(OPENCM3STM)/st_usbfs_v1.c

	LDSCRIPT = $(OPENCM3PATH)/stm32/f1/stm32f103.ld

	OPENOCD_CFG := _build_tools/openocd/stm32f1x.cfg
	
	OPENOCD_UNLOCK := -c "init" c "stm32f1x unlock 0"
	#OPENOCD_UNLOCK := -c "init" -c "reset" -c "halt 0" -c "mww 0x40022004 0x45670123" -c "mww 0x40022004 0xcdef89ab" -c "mww 0x40022008 0x45670123" -c "mww 0x40022008 0xcdef89ab" -c "mdw 0x40022010" -c "mdh 0x1ffff800 8" -c "mww 0x40022010 0x0210" -c "mwh 0x1ffff800 0x5aa5" -c "mdw 0x4002200c"

	OPENOCD_ERASE :=  -c "init" -c "reset" -c "halt 0" -c "stm32f1x mass_erase 0"

else ifeq ($(COREFILES_FAMILY), f4)
	C_FILES += $(wildcard $(OPENCM3STM)/f4/*.c)
	C_FILES += $(wildcard $(OPENCM3COMMON)/*_f24.c)
	C_FILES += $(wildcard $(OPENCM3COMMON)/*_f47.c)
	C_FILES += $(wildcard $(OPENCM3COMMON)/*_f0234.c)
	C_FILES += $(wildcard $(OPENCM3COMMON)/*_f234.c)
	C_FILES += $(OPENCM3COMMON)/flash_common_idcache.c

	LDSCRIPT = $(OPENCM3PATH)/stm32/f4/stm32f405x6.ld

	#OPENOCD_CFG := board/stm32f4discovery.cfg
	OPENOCD_CFG := _build_tools/openocd/stm32f4discovery.cfg

	OPENOCD_UNLOCK := -c "init" -c "reset" -c "halt 0" -c "stm32f4x unlock 0"
	OPENOCD_ERASE :=  -c "init" -c "reset" -c "halt 0" -c "stm32f4x mass_erase 0"
endif

OPENOCD_PROGRAM := -c "program __build/$(TARGET).elf verify; reset; exit;"

C_FILES += $(call rwildcard, $(OPENCM3USB), *.c)


######################################################################
# Create file arrays for the c++ and c compiler, and linker

INC_DIRS := $(sort $(dir $(call rwildcard, $(BOOTLOADER_PATH), *.h)))
INC_DIRS += $(sort $(dir $(call rwildcard, $(OPENCM3STM), *.h)))


# Object file (and assembly file) path array
SOURCE_OBJS := 	$(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o)

OBJECTS := $(foreach obj, $(SOURCE_OBJS), $(BUILDDIR)/$(obj))

LIBRARIES    := $(foreach lib, $(INC_DIRS), -I$(lib))
LD_LIBRARIES := $(foreach lib, $(INC_DIRS), -L$(lib))


######################################################################
# Create build rules

all: $(TARGET).hex $(TARGET).bin $(TARGET).asm

build: $(TARGET).elf

upload: $(TARGET).bin
	openocd -c 'debug_level 2' -f interface/stlink-v2.cfg -c 'set CPUTAPID $(CPUID)' -f $(OPENOCD_CFG) $(OPENOCD_ERASE) $(OPENOCD_UNLOCK) -c "exit"
	#openocd -c 'debug_level 2' -f interface/stlink-v2.cfg -c 'set CPUTAPID $(CPUID)' -f $(OPENOCD_CFG) $(OPENOCD_UNLOCK) -c "exit"
	openocd -c 'debug_level 2' -f interface/stlink-v2.cfg -c 'set CPUTAPID $(CPUID)' -f $(OPENOCD_CFG)  $(OPENOCD_PROGRAM)

read-options:
	openocd -f interface/stlink-v2.cfg -c 'set CPUTAPID $(CPUID)' -f target/stm32f1x.cfg init -c "halt" -c "stm32f1x options_read 0"


$(BUILDDIR)/%.o: %.c
	@echo "Building file $(notdir $<)"
	@mkdir -p "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(LIBRARIES) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.cpp
	@echo "Building file $(notdir $<)"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARIES) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.S
	@echo "Building file $(notdir $<)"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARIES) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.ino
	@echo "Building file $(notdir $<)"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARIES) -o "$@" -x c++ -include Arduino.h -c "$<"

$(BUILDDIR)/%.o: %.s
	@echo "Building file $(notdir $<)"
	@mkdir -p "$(dir $@)"
	@$(CXX) -x assembler-with-cpp $(CPPFLAGS) $(CFLAGS) $(LIBRARIES) -MD -o "$@" -c "$<"

$(TARGET).elf: $(OBJECTS)
	@echo "Linking $(BUILDDIR)/$@"
	@$(CC) $(LDFLAGS) $(OBJECTS) $(LD_LIBRARIES) -T$(LDSCRIPT) -o $(BUILDDIR)/"$@" 

%.hex: %.elf
	@echo "Making HEX $(BUILDDIR)/$@"
	@$(OBJCOPY) -O ihex -R .eeprom $(BUILDDIR)/"$<" $(BUILDDIR)/"$@"

%.bin: %.elf
	@echo "Making BIN $(BUILDDIR)/$@"
	@$(SIZE) $(BUILDDIR)/"$<"
	@$(OBJCOPY) -O binary $(BUILDDIR)/"$<" $(BUILDDIR)/"$@"

%.asm: %.elf %.bin
	@echo "Dumping $(BUILDDIR)/$@" 
	@$(DUMP) -d $(BUILDDIR)/"$<" > $(BUILDDIR)/$(TARGET)_elf.s
	@$(DUMP) -marm -Mforce-thumb -d $(BUILDDIR)/"$<" > $(TARGET)_bin.asm


# Compiler generated dependency info
-include $(OBJECTS:.o=.d)

clean:
	@echo Cleaning "$(abspath $(BUILDDIR))"
	@rm -rf "$(abspath $(BUILDDIR))"
	@echo Done!

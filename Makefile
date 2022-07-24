######################################################################
# STM32Fx0x Arduino Makefile
# (c) j0ule 2022, All Rights Reserved

# This is part of an STM32 Framework organized by Adam Munich for 
# hobby projects. It is a work in progress. 
# It is for non-commercial use only, unless explicit permission 
# and terms of use are given.

# The name of your project (used to name the compiled .hex file)
TARGET = fw

# Directories
PROJECTSDIR = projects

# Internal
FRAMEWORKDIR = system32
LIBRARIESDIR = $(FRAMEWORKDIR)/libraries

# ==================================================================
# Family
ifndef MCU_FAMILY
	MCU_FAMILY := f103cb
endif

# Path of project source, set from command line
ifndef SRC
    $(error SRC is not defined, there is nothing to build...)
else
    $(warning Building SRC "$(SRC)")
endif

SOURCEPATH := $(PROJECTSDIR)/$(SRC)

# moto, or inverter, uses STM32F4
ifeq ($(SRC),$(filter $(SRC),moto))
	MCU_FAMILY := f40x
	ODRIVE_CODEBASE := 1
endif

# Check if the file is buildable
#ifneq ("$(wildcard $(PROJECTSDIR)/$(SRC)/*.ino)","")
#	SOURCEPATH := $(PROJECTSDIR)/$(SRC)
#else
#    $(error No '.ino' file found in source path, there is nothing to build...)
#endif

# Some libraries will require this to be defined
ARDUINO = 110035

USBD_USE_CDC = 1


# ==================================================================
# Configurable GCC Options
OPTIONS := 

# What microcontroller?
ifeq ($(MCU_FAMILY), f103cb)
    $(warning Building for MCU_FAMILY "f103cb")
	COREFILES_FAMILY    := f1
	CPUID               := 0x1ba01477
	FLASH_SIZE          := 131072
	RAM_SIZE            := 20480
	OPTIONS             += -DARDUINO_BLUEPILL_F103CB
	OPTIONS				+= -DSTM32F103xB
	VARIANT 			:= variant_PILL_F103Cx
	VARIANTPATH 		:= $(FRAMEWORKDIR)/Arduino_Core_STM32/variants/STM32F1xx/F103C8T_F103CB(T-U)
	OPTIONS 			+= -DVARIANT_H="<$(VARIANT).h>"

else ifeq ($(MCU_FAMILY), f103rc)
    $(warning Building for MCU_FAMILY "f103rc")
	COREFILES_FAMILY    := f1
	CPUID               := 0x1ba01477
	FLASH_SIZE          := 262144
	RAM_SIZE            := 65536
	OPTIONS             += -DARDUINO_BLUEBUTTON_F103RCT
	#OPTIONS            += -DARDUINO_GENERIC_F103RCTX
	OPTIONS				+= -DSTM32F103xE
	VARIANT 			:= variant_BLUEBUTTON_F103RCT
	VARIANTPATH 		:= $(FRAMEWORKDIR)/Arduino_Core_STM32/variants/STM32F1xx/F103R(C-D-E)T
	OPTIONS 			+= -DVARIANT_H="<$(VARIANT).h>"

else
    $(warning Building for MCU_FAMILY "f40x")
	COREFILES_FAMILY    := f4
	CPUID               := 0x2ba01477
	FLASH_SIZE 			:= 1048576
	RAM_SIZE            := 196608
	OPTIONS             += -DARDUINO_GENERIC_F405RGTX
	OPTIONS				+= -DSTM32F405xx
	VARIANT 			:= variant_generic
	VARIANTPATH 		:= $(FRAMEWORKDIR)/Arduino_Core_STM32/variants/STM32F4xx/F405RGT_F415RGT
	OPTIONS 			+= -DVARIANT_H="<$(VARIANT).h>"
endif

# Ignore CPUID
CPUID := 0

ifeq ($(PNTP_TCP), 1)
    $(warning Building with PNTP TCP Support")
	USE_PNTP 			:= 1
	OPTIONS				+= -DINCLUDE_PNTP
	OPTIONS				+= -DPNTP_USING_ETH0
	OPTIONS				+= -DPNTP_USING_TTY
endif

ifeq ($(PNTP_TTY), 1)
    $(warning Building with PNTP TTY Support")
	USE_PNTP 			:= 1
	OPTIONS				+= -DINCLUDE_PNTP
	OPTIONS				+= -DPNTP_USING_TTY
endif

# More Options
OPTIONS += -DHAL_UART_MODULE_ENABLED 
OPTIONS += -DHAL_PCD_MODULE_ENABLED

OPTIONS += -DARDUINO_ARCH_STM32
OPTIONS += -DUSBCON 

ifeq ($(USBD_USE_CDC), 1)
	OPTIONS += -DUSBD_USE_CDC 
	OPTIONS += -DUSBD_CDC_USE_SINGLE_BUFFER
endif

# CPU Specific Modifiers
ifeq ($(COREFILES_FAMILY), f1)
	CPU_VARIANT := STM32F1xx
	OPTIONS     += -DBOARD_NAME="\"BLUEPILL_F103C8\"" 
	OPTIONS     += -DSTM32F10X_MD 
	OPTIONS     += -D__STM32F1__
	OPTIONS     += -DARDUINO_ARCH_STM32F1

	BOOTLOADER_SIZE := 0x2800

else ifeq ($(COREFILES_FAMILY), f4)
	CPU_VARIANT := STM32F4xx
	OPTIONS     += -DBOARD_NAME="\"DISCO_F407VG\"" 
	OPTIONS     += -D'__UNALIGNED_UINT32_READ(addr)=(*((const __packed uint32_t *)(addr)))'
	OPTIONS     += -D'__UNALIGNED_UINT32_WRITE(addr, val)=((*((__packed uint32_t *)(addr))) = (val))'
	OPTIONS     += -D__STM32F4__
	#OPTIONS    += -DARDUINO_ARCH_STM32F4

	BOOTLOADER_SIZE := 0x4000
endif

ifdef CPU_VARIANT
	OPTIONS += -D$(CPU_VARIANT)
endif

# USB Data
USB_VENDOR_ID = 1337
USB_PRODUCT_ID = c0de

# Special meta settings
ifdef META
	OPTIONS += -DUSE_META_SETTINGS

	# USB IDs
	USB_VENDOR_ID = 0483
	USB_PRODUCT_ID = DF11

	TEMP_META_HACK = "META_V5=1"
endif

OPTIONS += -DUSBD_VID=0x$(USB_VENDOR_ID) 
OPTIONS += -DUSBD_PID=0x$(USB_PRODUCT_ID)
OPTIONS += -DUSB_PRODUCT_STRING="\"x42 Node\"" 
OPTIONS += -DUSB_MANUFACTURER_STRING="\"Civil Electric\""

ifdef ODRIVE_CODEBASE
	# Firmware version macros
	FW_VERSION_MAJOR = 0
	FW_VERSION_MINOR = 4
	FW_VERSION_REVISION = 7
	FW_VERSION_UNRELEASED = 0

	# Hardware version macros
	HW_VERSION_MAJOR = 3
	HW_VERSION_MINOR = 5
	HW_VERSION_VOLTAGE = 48

	# Configurable Options
	OPTIONS = -DSTM32F405xx
	OPTIONS += -ggdb -DUSE_HAL_DRIVER
	OPTIONS += -DFW_VERSION_MAJOR=$(FW_VERSION_MAJOR) -DFW_VERSION_MINOR=$(FW_VERSION_MINOR)
	OPTIONS += -DFW_VERSION_REVISION=$(FW_VERSION_REVISION) -DFW_VERSION_UNRELEASED=$(FW_VERSION_UNRELEASED)
	OPTIONS += -DHW_VERSION_MAJOR=$(HW_VERSION_MAJOR) -DHW_VERSION_MINOR=$(HW_VERSION_MINOR) -DHW_VERSION_VOLTAGE=$(HW_VERSION_VOLTAGE)
	OPTIONS += -DCDC_BAUD_RATE=921600

	#OPTIONS += -DUSE_MOTO_PINS
	#OPTIONS += -DUSE_HIGH_VOLTAGE
	OPTIONS += -DUSE_SINGLE_AXIS

	OPTIONS += -DUSB_PROTOCOL_NATIVE_STREAM_BASED
endif

# Use bootloader or not (now a command line option)
ifneq ($(NO_BOOTLOADER), 1)
	OPTIONS += -DVECT_TAB_OFFSET=$(BOOTLOADER_SIZE)
else
	BOOTLOADER_SIZE := 0
endif

# Set arduino define if given
ifdef ARDUINO
    OPTIONS += -DARDUINO=$(ARDUINO)
endif

######################################################################
# Location of paths, libraries

# Directory to build in
BUILDDIR = $(abspath $(CURDIR)/__build)

# Directory to project in
RELEASEDIR = $(BUILDDIR)/$(SRC)

# Where bootloader keys are stored
KEYDIR_BASE = $(BUILDDIR)/_bootloader_keys

# RDP enabled?
ifeq ($(USE_RDP), 1)
	BOOTLOADER_READOUT_PROTECT := USE_RDP=1
	DEBUG := 0
else
	DEBUG := 1
	USE_RDP := 0
endif

# Encrypt?
ifeq ($(USE_ENCRYPTION), 1)
	BOOT_KEY_FILE := $(KEYDIR_BASE)/$(SRC)/bootkey.h
	USE_BOOTLOADER_KEY_FILE := USE_BOOTLOADER_KEY_FILE=$(BOOT_KEY_FILE)
else 
	USE_BOOTLOADER_KEY_FILE :=
	BOOT_KEY_FILE :=
endif

# Path location for arduino core
PATH_ARDUINO_CORE_STM32 = $(FRAMEWORKDIR)/Arduino_Core_STM32/cores/arduino

# Path location for arduino libraries
PATH_LIBRARY_SPI        = $(FRAMEWORKDIR)/Arduino_Core_STM32/libraries/SPI/src
PATH_LIBRARY_WIRE       = $(FRAMEWORKDIR)/Arduino_Core_STM32/libraries/Wire/src
PATH_LIBRARY_SERVO      = $(FRAMEWORKDIR)/Arduino_Core_STM32/libraries/Servo/src
PATH_LIBRARY_KEYBOARD   = $(FRAMEWORKDIR)/Arduino_Core_STM32/libraries/Keyboard/src
PATH_LIBRARY_MOUSE      = $(FRAMEWORKDIR)/Arduino_Core_STM32/libraries/Mouse/src
PATH_LIBRARY_SRCWRAPPER = $(FRAMEWORKDIR)/Arduino_Core_STM32/libraries/SrcWrapper/src

# External libraries
PATH_LIBRARY_ETHERNET   = $(LIBRARIESDIR)/Ethernet/src
#PATH_LIBRARY_ETHERNET   = $(LIBRARIESDIR)/Ethernet_Generic/src
#PATH_LIBRARY_BONJOUR    = $(LIBRARIESDIR)/MDNS_Generic/src
PATH_LIBRARY_BONJOUR    = $(LIBRARIESDIR)/ArduinoMDNS

PATH_ST_HAL             = $(FRAMEWORKDIR)/Arduino_Core_STM32/system/Drivers/$(CPU_VARIANT)_HAL_Driver
PATH_SYSTEM_HEADERS     = $(FRAMEWORKDIR)/Arduino_Core_STM32/system/Drivers/CMSIS/Device/ST/$(CPU_VARIANT)/Include
PATH_STARTUP_FILES      = $(FRAMEWORKDIR)/Arduino_Core_STM32/system/Drivers/CMSIS/Device/ST/$(CPU_VARIANT)/Source/Templates/gcc
PATH_ST_HAL_CONF	    = $(FRAMEWORKDIR)/Arduino_Core_STM32/system/$(CPU_VARIANT)
PATH_MIDDLEWARES	    = $(FRAMEWORKDIR)/Arduino_Core_STM32/system/Middlewares/ST
PATH_CMSIS              = $(FRAMEWORKDIR)/CMSIS

# Path location for FreeRTOS
PATH_FREERTOS           = $(FRAMEWORKDIR)/STM32FreeRTOS/src

# Patches to arduino core
PATH_PATCHES            = $(FRAMEWORKDIR)/Arduino_Core_STM32_patches

# Special config for ODRIVE_CODEBASE
ifdef ODRIVE_CODEBASE
	VARIANTPATH          := $(SOURCEPATH)/Board/v3
	PATH_LIBRARY_PNTP    := $(LIBRARIESDIR)/lib_powernet/protocol
	NVMPATH              := $(LIBRARIESDIR)/lib_powernet/nvm
else
	PATH_LIBRARY_PNTP    := $(LIBRARIESDIR)/lib_powernet
endif

# DSP Library
PATH_LIBRARY_DSP    	 := $(LIBRARIESDIR)/lib_dsp


# Target Path
TARGET_PATH = $(TARGET)


######################################################################
# Compiler configuration

# Compiler flags for C and C++
CPPFLAGS := $(OPTIONS) 
CPPFLAGS += -mthumb
CPPFLAGS += -fno-exceptions
CPPFLAGS += -ffunction-sections
CPPFLAGS += -fdata-sections
CPPFLAGS += -MMD
CPPFLAGS += -ffast-math 
CPPFLAGS += -fno-math-errno
CPPFLAGS += -Werror=return-type
CPPFLAGS += -Wall -Wdouble-promotion -Wfloat-conversion

# Special compiler flags for ODRIVE_CODEBASE
ifdef ODRIVE_CODEBASE

	CPPFLAGS += -Og 
	CPPFLAGS += -g
	CPPFLAGS += -fno-finite-math-only
	CPPFLAGS += -Wformat=0
	CPPFLAGS += -D__weak="__attribute__((weak))" 
	CPPFLAGS += -D__packed="__attribute__((__packed__))" 

	CPPFLAGS += -fbranch-count-reg 
	CPPFLAGS += -fdse  -fif-conversion  -fif-conversion2  
	CPPFLAGS += -finline-functions-called-once 
	CPPFLAGS += -fmove-loop-invariants  -fssa-phiopt 
	CPPFLAGS += -ftree-bit-ccp  -ftree-dse  -ftree-pta  -ftree-sra
else 
	CPPFLAGS += -Os 
endif

# Special compiler flags for STM32F1
ifeq ($(COREFILES_FAMILY), f1)

	CPPFLAGS += -mcpu=cortex-m3
	CPPFLAGS += -mfloat-abi=softfp

# Special compiler flags for STM32F4
else ifeq ($(COREFILES_FAMILY), f4)

	CPPFLAGS += -mcpu=cortex-m4
	CPPFLAGS += -mfpu=fpv4-sp-d16
	CPPFLAGS += -mfloat-abi=hard
endif

# Debug symbols?
ifeq ($(DEBUG), 1)
	CPPFLAGS += -g
endif

# Compiler flags for C++ only
CXXFLAGS := -felide-constructors 
CXXFLAGS += -fno-exceptions 
CXXFLAGS += -fno-rtti 
CXXFLAGS += -fno-threadsafe-statics 
CXXFLAGS += -fext-numeric-literals
CXXFLAGS += -std=c++17
CXXFLAGS += -Wno-register -Wno-bool-compare

# Compiler flags for C only
CFLAGS := -std=c99


######################################################################
# Linker configuration

# Linker flags
LDFLAGS := -mthumb 
LDFLAGS += -g
LDFLAGS += -lm
LDFLAGS += -lc 
LDFLAGS += -lnosys 
LDFLAGS += -lstdc++ 
LDFLAGS += -specs=nano.specs -specs=nosys.specs 
LDFLAGS += -u _printf_float
LDFLAGS += -Wl,--cref 
LDFLAGS += -Wl,--check-sections
LDFLAGS += -Wl,-gc-sections
LDFLAGS += -Wl,--defsym=LD_FLASH_OFFSET=$(BOOTLOADER_SIZE)
LDFLAGS += -Wl,--entry=Reset_Handler 
LDFLAGS += -Wl,--unresolved-symbols=report-all 
#LDFLAGS += -Wl,--warn-common
LDFLAGS += -Wl,-Map,$(BUILDDIR)/$(TARGET_PATH).map
LDFLAGS += -Wl,--start-group 
LDFLAGS += -Wl,--no-whole-archive 
LDFLAGS += -Wl,--end-group

# Special linker flags for STM32F1
ifeq ($(COREFILES_FAMILY), f1)

	LDFLAGS += -mcpu=cortex-m3 
	LDFLAGS += -Wl,--defsym=LD_MAX_DATA_SIZE=$(RAM_SIZE)
	LDFLAGS += -Wl,--library=arm_cortexM3l_math

	LDSCRIPT := $(FRAMEWORKDIR)/_build_tools/linker_scripts/stm32f1xx.ld

# Special linker flags for STM32F4
else ifeq ($(COREFILES_FAMILY), f4)
	
	LDFLAGS += -mcpu=cortex-m4
	LDFLAGS += -mfpu=fpv4-sp-d16
	LDFLAGS += -mfloat-abi=hard
	LDFLAGS += -Wl,--defsym=LD_MAX_DATA_SIZE=$(RAM_SIZE)
	LDFLAGS += -Wl,--library=arm_cortexM4lf_math

	LDSCRIPT := $(FRAMEWORKDIR)/_build_tools/linker_scripts/stm32f4xx.ld
endif

# Special linker flags for ODRIVE_CODEBASE
ifdef ODRIVE_CODEBASE

	LDFLAGS += -L"$(VARIANTPATH)/Drivers/CMSIS/Lib"
	LDFLAGS += -Wl,--undefined=uxTopUsedPriority

	LDSCRIPT := $(FRAMEWORKDIR)/_build_tools/linker_scripts/STM32F405RGTx_FLASH.ld
endif

LDFLAGS += -Wl,--defsym=LD_MAX_SIZE=$(FLASH_SIZE)

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
READELF = arm-none-eabi-readelf
NM = arm-none-eabi-nm


######################################################################
# Automatically create lists of the sources and objects
rwildcard = $(foreach d,$(wildcard $1*),$(call rwildcard,$d/,$2) $(filter $(subst *,%,$2),$d))

# Source for ODRIVE_CODEBASE (slowly... we will merge as many dependencies as we can)
ifdef ODRIVE_CODEBASE
	# Source
	C_FILES := $(call rwildcard, $(SOURCEPATH), *.c)
	C_FILES += $(call rwildcard, $(NVMPATH), *.c)
	C_FILES := $(filter-out $(wildcard $(SOURCEPATH)/Board/v3/Src/prev_board_ver/*.c), $(C_FILES))

	CPP_FILES := $(call rwildcard, $(SOURCEPATH), *.cpp)

	CPP_FILES += $(call rwildcard, $(NVMPATH), *.cpp)

	CPP_FILES := $(filter-out $(wildcard $(SOURCEPATH)/fibre/test/*.cpp), $(CPP_FILES))
	CPP_FILES := $(filter-out $(wildcard $(SOURCEPATH)/fibre/cpp/posix*.cpp), $(CPP_FILES))

	C_FILES   += $(call rwildcard, $(PATH_LIBRARY_PNTP), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_PNTP), *.cpp)

	# Assembly
	ASM_FILES := $(call rwildcard, $(SOURCEPATH), *.s)

	INC_DIRS := $(sort $(PROJECTSDIR)/$(dir $(call rwildcard, $(SOURCEPATH), *)))
	INC_DIRS += $(sort $(PROJECTSDIR)/$(dir $(call rwildcard, $(NVMPATH), *)))
	INC_DIRS += $(sort $(PROJECTSDIR)/$(dir $(call rwildcard, $(PATH_LIBRARY_PNTP), *)))


# If not ODRIVE_CODEBASE, source for everything else...
else
	# Source
	INO_FILES := $(filter-out %examples%, $(wildcard $(SOURCEPATH)/*.ino))
	C_FILES   := $(filter-out %examples%, $(call rwildcard, $(SOURCEPATH), *.c))
	CPP_FILES := $(filter-out %examples%, $(call rwildcard, $(SOURCEPATH), *.cpp))

	# ST HAL
	ifeq ($(USE_OLD_CORE), 1)
		C_FILES += $(filter-out %template.c, $(call rwildcard, $(PATH_ST_HAL), *.c))
		CPP_FILES += $(call rwildcard, $(PATH_ST_HAL), *.cpp)
	endif

	# Arduino std libraries use max 2 to 3 directory levels
	C_FILES   += $(call rwildcard, $(PATH_LIBRARY_SPI), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_SPI), *.cpp)
	C_FILES   += $(call rwildcard, $(PATH_LIBRARY_WIRE), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_WIRE), *.cpp)
	C_FILES   += $(call rwildcard, $(PATH_LIBRARY_SERVO), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_SERVO), *.cpp)
	C_FILES   += $(call rwildcard, $(PATH_LIBRARY_SRCWRAPPER), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_SRCWRAPPER), *.cpp)

	C_FILES += $(call rwildcard, $(PATH_LIBRARY_ETHERNET), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_ETHERNET), *.cpp)
	C_FILES += $(call rwildcard, $(PATH_LIBRARY_BONJOUR), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_BONJOUR), *.cpp)

	ifeq ($(USBD_USE_HID_COMPOSITE), 1)
	C_FILES   += $(call rwildcard, $(PATH_LIBRARY_KEYBOARD), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_KEYBOARD), *.cpp)
	C_FILES   += $(call rwildcard, $(PATH_LIBRARY_MOUSE), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_MOUSE), *.cpp)
	endif

	# FreeRTOS
	C_FILES += $(call rwildcard, $(PATH_FREERTOS), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_FREERTOS), *.cpp)
	C_FILES += $(call rwildcard, $(PATH_CMSIS), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_CMSIS), *.cpp)

	ifeq ($(USE_PNTP), 1)
	# Comms
	C_FILES += $(call rwildcard, $(PATH_LIBRARY_PNTP), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_PNTP), *.cpp)
	endif

	# DSP
	C_FILES += $(call rwildcard, $(PATH_LIBRARY_DSP), *.c)
	CPP_FILES += $(call rwildcard, $(PATH_LIBRARY_DSP), *.cpp)

	# Core
	C_FILES += $(call rwildcard, $(PATH_ARDUINO_CORE_STM32), *.c)
	C_FILES += $(wildcard $(VARIANTPATH)/*.c)
	CPP_FILES += $(call rwildcard, $(PATH_ARDUINO_CORE_STM32), *.cpp)
	CPP_FILES += $(wildcard $(VARIANTPATH)/*.cpp)

	# Remove files we patched
	C_FILES := $(filter-out %usbd_ep_conf.c, $(C_FILES))
	C_FILES := $(filter-out %cdc_queue.c, $(C_FILES))
	C_FILES := $(filter-out %usbd_cdc.c, $(C_FILES))
	C_FILES := $(filter-out %usbd_cdc_if.c, $(C_FILES))

	#C_FILES := $(filter-out %hw_config.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_desc.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_cdc.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_cdc_if.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_hid_composite.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_hid_composite_if.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_audio.c, $(C_FILES))

	# Files we Patched
	C_FILES += $(call rwildcard, $(PATH_PATCHES), *.c)

	# Hack to fix someone's bad hack
	#C_FILES := $(filter-out %usbd_core.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_ioreq.c, $(C_FILES))
	#C_FILES := $(filter-out %usbd_ctlreq.c, $(C_FILES))

	# Assembly
	ASM_FILES := $(call rwildcard, $(PATH_ARDUINO_CORE_STM32), *.s)

	# Entry
	S_FILES := $(FRAMEWORKDIR)/Arduino_Core_STM32/cores/arduino/stm32/startup_stm32yyxx.S

	######################################################################
	# Create file arrays for the c++ and c compiler, and linker

	# Program
	INC_DIRS := $(sort $(PROJECTSDIR)/$(dir $(call rwildcard, $(SOURCEPATH), *)))

	# Core
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_ARDUINO_CORE_STM32), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_ST_HAL), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_ST_HAL_CONF), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_SYSTEM_HEADERS), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_STARTUP_FILES), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_MIDDLEWARES), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_FREERTOS), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_CMSIS), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_PATCHES), *)))

	# Libs
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_SPI), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_WIRE), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_SERVO), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_SRCWRAPPER), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_ETHERNET), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_BONJOUR), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_PNTP), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_DSP), *)))

	ifeq ($(USBD_USE_HID_COMPOSITE), 1)
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_KEYBOARD), *)))
	INC_DIRS += $(sort $(dir $(call rwildcard, $(PATH_LIBRARY_MOUSE), *)))
	endif

	# Board variant
	INC_DIRS += $(sort $(dir $(wildcard $(VARIANTPATH)/*)))
	INC_DIRS += $(sort $(dir $(wildcard $(CUSTOM_VARIANTPATH)/*)))

endif 


######################################################################
# Create file arrays for the c++ and c compiler, and linker

# Object file (and assembly file) path array
SOURCE_OBJS := $(S_FILES:.S=.o) $(ASM_FILES:.s=.o) $(C_FILES:.c=.o) $(CPP_FILES:.cpp=.o) $(INO_FILES:.ino=.o)

OBJECTS := $(foreach obj, $(SOURCE_OBJS), $(BUILDDIR)/$(obj))

# Include paths for header files and libraries
LIBRARIES := $(foreach lib, $(INC_DIRS), -I"$(lib)")
LD_LIBRARIES := $(foreach lib, $(INC_DIRS), -L"$(lib)")

# Whitespace
space := $(subst ,, )


######################################################################
# Create build rules

build: $(TARGET_PATH).elf $(TARGET_PATH).hex $(TARGET_PATH).bin $(TARGET_PATH).asm

all: build

#upload: $(TARGET_PATH).elf
#	openocd -f interface/stlink-v2.cfg -f target/stm32f1x.cfg -c "program $(BUILDDIR)/$(TARGET_PATH).elf verify reset exit"

#upload-alt: $(TARGET_PATH).elf
#	openocd -f interface/stlink-v2.cfg -c 'set CPUTAPID 0x2ba01477' -f target/stm32f1x.cfg -c "program $(BUILDDIR)/$(TARGET_PATH).elf verify reset exit"

#upload-dfu:  $(TARGET_PATH).bin
#	dfu-util -a 0 -s 0x08002000:leave -D $(BUILDDIR)/$(TARGET_PATH).bin

readback-f1:
	openocd -f interface/stlink.cfg -c 'set CPUTAPID $(CPUID)' -f target/stm32f1x.cfg -c "init" -c "reset init" -c "dump_image $(SRC).bin 0x8000000 $(FLASH_SIZE)" -c "exit"

define build-key
	@echo "\nMAKE: Generating boot key...\n"
	@python3 ./$(FRAMEWORKDIR)/_build_tools/keygen.py $(KEYDIR_BASE) $(PROJECTSDIR) $(SRC)
endef

define build-upload-bootloader
	@echo "\nMAKE: Building bootloader...\n"
	@(cd ./$(FRAMEWORKDIR)/bootloader && $(MAKE) $(USE_BOOTLOADER_KEY_FILE) USB_VENDOR_ID=$(USB_VENDOR_ID) USB_PRODUCT_ID=$(USB_PRODUCT_ID) MCU_FAMILY=$(MCU_FAMILY) $(TEMP_META_HACK) $(BOOTLOADER_READOUT_PROTECT) CPUID=$(CPUID) clean upload)	
	@ sleep 1
endef

define snappack
	@echo "\nMAKE: Packing $(TARGET_PATH).bin...\n"
	@python3 ./$(FRAMEWORKDIR)/_build_tools/snappack.py $(BUILDDIR)/$(TARGET_PATH).bin $(RELEASEDIR)/$(TARGET_PATH).dfu $(BOOTLOADER_SIZE) $(BOOT_KEY_FILE)
endef

define upload-dfuse
	@echo "\nMAKE: Uploading $(TARGET_PATH).dfu...\n"
	@-dfu-util --device $(USB_VENDOR_ID):$(USB_PRODUCT_ID) -a 0 -s 0x08000000:leave -D $(RELEASEDIR)/$(TARGET_PATH).dfu
endef

define upload-dfuse-v
	@echo "\nMAKE: Uploading $(TARGET_PATH).dfu...\n"
	dfu-util --device $(USB_VENDOR_ID):$(USB_PRODUCT_ID) -a 0 -s 0x08000000:leave -D $(RELEASEDIR)/$(TARGET_PATH).dfu -v -v
endef

define bell
	@echo "\a"
endef

upload-bootloader:
	$(build-key)
	$(build-upload-bootloader)

upload-dfu: $(TARGET_PATH).bin
	$(snappack)
	$(upload-dfuse)
	$(bell)

upload-verbose: $(TARGET_PATH).bin
	$(snappack)
	$(upload-dfuse-v)

upload-all: $(TARGET_PATH).bin
	$(build-key)
	$(snappack)
	$(build-upload-bootloader)
	$(upload-dfuse)
	$(bell)

#upload-dfu-aes:  $(TARGET_PATH).bin
#	@echo "\nMAKE: Locking down $(TARGET_PATH).bin...\n"
#	@./_build_tools/lockdown.py $(BUILDDIR)/$(TARGET_PATH).bin $(RELEASEDIR)/$(TARGET_PATH).dfu $(BOOTLOADER_SIZE)
#	@echo "\nMAKE: Uploading $(TARGET_PATH).dfu...\n"
#	@dfu-util -a 0 -s 0x08000000:leave -D $(RELEASEDIR)/$(TARGET_PATH).dfu

#upload-dfu-aes-test:  $(TARGET_PATH).bin
#	./_build_tools/lockdown.py ./_build_tools/test.bin ./_build_tools/test.dfu $(BOOTLOADER_SIZE)
#	dfu-util -a 0 -s 0x08000000:leave -D ./_build_tools/test.dfu -v

$(BUILDDIR)/%.o: %.c
	@echo "\nMAKE: Building file $<"
	@mkdir -p "$(dir $@)"
	@$(CC) $(CPPFLAGS) $(CFLAGS) $(LIBRARIES) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.cpp
	@echo "\nMAKE: Building file $<"
	@mkdir -p "$(dir $@)" 
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARIES) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.S
	@echo "\nMAKE: Building file $<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARIES) -o "$@" -c "$<"

$(BUILDDIR)/%.o: %.ino
	@echo "\nMAKE: Building file $<"
	@mkdir -p "$(dir $@)"
	@$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARIES) -o "$@" -x c++ -include Arduino.h -c "$<"

$(BUILDDIR)/%.o: %.s
	@echo "\nMAKE: Building file $<"
	@mkdir -p "$(dir $@)"
	@$(CXX) -x assembler-with-cpp $(CPPFLAGS) $(CFLAGS) $(LIBRARIES) -MD -o "$@" -c "$<"

$(TARGET_PATH).elf: $(OBJECTS)
	@echo "\nMAKE: Linking $(TARGET_PATH).elf..."
	@$(CC) -o "$(BUILDDIR)/$@" "$(subst $(space)," ",$^)" $(LDFLAGS) $(LD_LIBRARIES) -T$(LDSCRIPT) 

%.hex: %.elf
	@echo "\nMAKE: Copying $(TARGET_PATH).hex..."
	@$(OBJCOPY) -O ihex -R .eeprom $(BUILDDIR)/"$<" $(BUILDDIR)/"$@"

%.bin: %.elf
	@echo "\nMAKE: Copying $(TARGET_PATH).bin..."
	@$(SIZE) $(BUILDDIR)/"$<"
	@$(OBJCOPY) -O binary $(BUILDDIR)/"$<" $(BUILDDIR)/"$@"

%.asm: %.elf %.bin
	@echo "\nMAKE: Dumping $(TARGET_PATH).xxx..." 
	@$(DUMP) -d -S $(BUILDDIR)/"$<" > $(BUILDDIR)/$(TARGET_PATH)_elf.asm
	@$(DUMP) -marm -Mforce-thumb -d -S $(BUILDDIR)/"$<" > $(BUILDDIR)/$(TARGET_PATH)_bin.asm

# Compiler generated dependency info
-include $(OBJECTS:.o=.d)

clean:
	@echo "\nMAKE: Cleaning $(abspath $(BUILDDIR))"
	@rm -rf "$(abspath $(BUILDDIR))"
	@echo "\t Done!"

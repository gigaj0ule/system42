# STM42: The answer to life, the universe, and everything STM32
# (c) ~j0ule 2019, 2020, 2021, All Rights Reserved

This is part of an STM32 Framework organized by ~j0ule for hobby projects. 
It is a work in progress. It is for non-commercial use only, unless explicit 
permission and terms of use are agreed upon in writing.

### Prerequisites

An install script is not written yet. You will need:

- make
- vscode
- openocd
- dfu-util
- "cortex-debug" add-on for vscode
- python3 (and whatever libraries it complains about wanting when you try to compile)
- arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 9.2.1 (see INSTALLING_GCC.md)

### Making a Project

To make a new project  you just copy the projects/demo folder and then see the files 
inside. You will find a few files:

- sketch.ino, which contains your "main" function
- sketch.hpp, which is your project header file

All of the other libraries are shared between all projects. Unless a library is 
absolutely shared between every project, please, keep it out of _shared_libs!

Clone "demo" folder into "myproject" (or whatever), then write your MCU code!


### Sketch.ino

Inside sketch.ino you will find some threads. One of them is named:
    
    static void worker_thread(void* arg)
    
This is where all the meat of your program should go. You can do whatever
you want inside this thread, but be sure to call os_delay(milliseconds) instead
of delay(milliseconds), or you will waste valuable CPU cycles. 

Never calling os_delay(milliseconds) will cause the CPU to lock up and freeze, 
so always call it at least once in a thread!


### Building a project

To build a project, make does all the work for you. You must tell make
what the name of the module firmware is via the SRC variable. In the case of
"demo", you have:

Goto the root firmware folder, then execute:

    sudo make SRC=demo clean upload-all

Where "demo" of course, is the firmware target.

This will automatically: 

- Mass erase the STM32
- (optionally) Generate an encryption key for the firmware
- (optionally) Encrypt the firmware file
- Upload the bootloader with openocd and st-link v2
- (optionally) Enable read-out protection and disable SWD/JTAG
- Upload the firmware over USB / DFU 

The firmware (.snap) file will be in ./__builds/<project> if you want 
to save it for the future. Otherwise it will be destroyed the next time 
make runs with "clean" enabled.

Sometimes you'd like to prevent the firmware from being read out of the MCU, 
if it stores private data fro example. You can enable "read out protection"
to (theoretically) prevent this, but, it doesn't actually prevent a determined
hacker in most cases. It just makes it slightly harder. 

To enable readout protection, use:

    sudo make SRC=demo USE_RDP=1 clean upload-all

There is also support for AES256 encryption, which prevents the firmware from 
being seen as plaintext. With AES enabled, a "boot_key" will be generated and 
built into the bootloader. Only firmwares encrypted with this key will be able 
to be loaded into the microcontroller. This is useful if for example, loading 
the wrong firmware into the MCU would cause an elctrical fire. Of course, if 
you lose this key, it will not be possible to update a firmware with DFU and 
you would need to flash a new bootloader onto the MCU manually.

To enable firmware encryption, use:

    sudo make SRC=demo USE_ENCRYPTION=1 clean upload-all


## Supported MCU

Right now, only the STM32F103 and STM32F405/7 MCUs are supported. By default
the build system will use the STM32F103C8T6. If you are building for something
else, use the following command:

    sudo make SRC=demo MCU_FAMILY=$FAMILY clean upload-all

Where $FAMILY is one of the following:

    f103c8
    f103rc
    f40x


### Debugging a project

You can use cortex-debug in vscode, but it only works when read out protection 
is turned off. SVD and configuration files should be already present for the 
STM32F1 family and the STM32F4 family in this repository.

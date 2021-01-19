# STM42: The answer to life, the universe, and everything STM32
# (c) Adam Munich 2020, All Rights Reserved

This is part of an STM32 Framework organized by Adam Munich for hobby projects. 
It is a work in progress. It is for non-commercial use only, unless explicit 
permission and usage terms of use are given.

### Prerequisites

An install script is not written yet. You will need:

- make
- vscode
- openocd
- dfu-util
- "cortex-debug" add-on for vscode
- python3 (and whatever libraries it complains about wanting when you try to compile)
- arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 9.2.1

### Making a Project

To make a new project  you just copy the projects/demo folder and then see the files 
inside. You will find a few files:

- sketch.ino, which contains your "main" function
- crash_handlers.cpp, which contain fault handlers

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

- Generate a key for the firmware
- Build the bootloader with this key
- Mass erase the STM32
- Upload the bootloader with openocd and st-link v2
- (optionally) Enable read-out protection and disable SWD/JTAG
- Encrypt the firmware file
- Upload the firmware over USB / DFU 

The firmware (.snap) file will be in ./__builds/<project> if you want 
to save it for the future. Otherwise it will be destroyed the next time 
make runs with clean

To enable readout protection, use

    sudo make SRC=demo RDP=1 clean upload-all


### Debugging a project

You can use cortex-debug in vscode

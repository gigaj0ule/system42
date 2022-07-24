# system42
### answer to life, universe, and everything m.c.u.

~j0ule 2019-2022 - alpha release

I am a multithreaded operating 
system for STM/CKS/GD 32 Bit 
microcontrollers.

I support arduino, st-hal, 
and freeRTOS simultaneously.

I support TTY and Ethernet
communication.

I have a BIOS that can accept 
new code via USB with dfu-util 
so you don't need a special 
programmer.

I am my own build system. Just 
type $ make.

I am solid. All my dependencies 
are included in-tree which means 
you will always be able to build 
and deploy me.

I can automate your labor by 
running tiny robots or even a 
whole village of them. These robot 
villages can help you live your 
best life on starship earth.


### Toolchain

An install script is not written yet. 

You will need:

- make
- vscode
- openocd
- dfu-util
- python3 (and: pycryptodome, )
- arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 9.2.1 (see INSTALLING_GCC.md)


### Starting a Project

To begin a new program you just copy 
the projects/demo folder and then view
the files inside. 

You will find:

- sketch.ino, which contains your "main" function

Clone "demo" folder into "projects/myproject" (or whatever), 
then write your MCU code!


### Building

To build a project, make does all 
the work for you. You must tell make
what the name of the module firmware 
is via the SRC variable. 

In the case of "projects/demo":

Goto the root firmware folder, 
then execute:

    sudo make SRC=demo clean upload-all

Where "demo" of course, is the 
firmware target.

This will automatically: 

- (optionally) Generate an encryption key for the firmware
- (optionally) Encrypt the firmware file
- Upload the bootloader with openocd and st-link v2
- (optionally) Enable read-out protection and disable SWD/JTAG
- Upload the firmware over USB / DFU 

The firmware (.snap) file will be 
in ./__builds/<project> if you want 
to save it for the future. Otherwise 
it will be destroyed the next time 
make runs with "clean" enabled.


### Threading

To create a thread:
    
    static void daughter_thread(void* arg)
   
You must call vTaskDekay(milliseconds) 
instead of delay(milliseconds), or
your CPU will waste useful cycles. 

Always call vTaskDelay() at least
once in a thread. If you don't, then
the thread will crash your core.


### USB Firmware Upgrade

After you have installed the BIOS
you can use     

    sudo make SRC=demo clean upload-dfu

To flash new firmware to the MCU 
over USB without a SWD programmer 
attached. This is great for pushing 
new firmware distributions out to 
other hackers who don't have debug
tools.

The DFU usb device id is 1337:c0de


### Employing ion beam researchers

Sometimes you'd like to prevent the 
firmware from being read out of the 
MCU, if it stores private data for
example. But it won't save your 
secret from a focused ion beam. 
Nothing will, really.

To enable readout protection, use:

    sudo make SRC=demo USE_RDP=1 clean upload-all

There is also support for AES256 
encryption, which prevents the 
firmware from being seen as 
plaintext. With AES enabled, 
a "boot_key" will be generated and 
built into the bootloader. 
Only firmwares encrypted with this 
key will be able to be loaded into 
the microcontroller. This is useful 
if for example, loading the wrong 
firmware into the MCU would cause 
an elctrical fire. Of course, if 
you lose this key, it will not be 
possible to update a firmware with 
DFU and you would need to flash a 
new bootloader onto the MCU manually.

To enable firmware encryption, use:

    sudo make SRC=demo USE_ENCRYPTION=1 clean upload-all

    
### Supported Cores

Right now, only the 32F103 and 
32F405/7 MCUs are supported. By 
default the build system will use 
the 32F103CBT6. If you are building 
for something else, use the 
following command:

    sudo make SRC=demo MCU_FAMILY=$FAMILY clean upload-all

Where $FAMILY is one of:

    f103cb
    f103rc
    f40x
    
It should be possible to compile for 
all other STM targets but the correct 
paths will need to be added to MAKEFILE
for them first.
    
The bootloader presently only supports
F1 and F4, since all families have
different flash memory controllers 
more #define logic must be added for 
other FMCs.

    
### Debugging a project

You can inspect variables in real 
time and pause Execution of your 
program with "cortex-debug" add-on 
for vscode. 

This is extremely useful. Like really
really makes you a better programmer. 

ST-Link V2 programmwrs have been tested with 
openocd and work great!

Debugging only works when read out 
protection is turned off. 

SVD and configuration files should 
be already present for the 32F1 
family and 32F4 family in this 
repository.

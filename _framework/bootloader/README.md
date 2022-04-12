# 32F DFUse BIOS

~j0ule 2020-2022 All Rights Reserved

This is part of an 32F operating 
system by ~j0ule for hobby projects. 
It is a work in progress.

This BIOS closely emulates the 
behavior of the built-in DFUse 
bootloader on most m32 devices, but 
provides extra features and 
customizations.

It expects that the firmware is 
built to execute with the IVT 
beginning after the last BIOS 
sector. Firmware linked for 
0x8000000 will not execute 
properly.

For 32F1, this is 0x8002000 (8k).

For 32F4, this is 0x8004000 (16k).

To flash a file to the program memory, 
pad the file with 8 or 16k of 0xFF 
and use:

    sudo dfu-util -a 0 -s 0x08000000:leave -D ./_build/sample/fw.bin

You must flash the file at address 
0x08000000. The BIOS will 
automatically skip over the padding 
in the file, which is expected to be 
the size of the BIOS sector itself. 
This padding can just be a bunch of 
0xFF. 

The BIOS has AES256 encryption 
support. If this is enabled, then you
must supply the BIOS with a 
file that is padded with the size 
of the bootloader, minus 16 bytes. 
The first 16 bytes are the AES CBC 
initialization vector used to encrypt 
the firmware file. See snappack.py 
for further insight.

You can use the following command 
to flash the encrypted file:

    sudo dfu-util -a 0 -s 0x08000000:leave -D ./_build/sample/fw.snap

Obviously, you must enable read out 
protection or your AES key stored 
in the bootloader will be leaked and 
the encryption is broken! As well, 
RDP should be enabled, otherwise 
the firmware can be read out in 
plaintext after successful 
decryption.


## Features

* Read-out protection
* Testing of a pin for forced DFU mode entry.
* Testing for RAM token for forced DFU mode entry.
* Application flash is wiped on download to prevent stub attacks.
* Option to disable DFUse upload to prevent firmware read-out.
* Firmware checksum validation (untested).
* AES256 CBC encryption.


## Booting into DFUse

You can enter the BIOS by writing 
0xB105F00D to the last four 
bytes of RAM and resetting the MCU. 
Alternatively, you can bring PB2 
HIGH and reset the MCU.


## Booting into Application:

If the following conditions are met, 
the BIOS will attempt to jump 
into the applicaton:

 * Program counter at IVT[0] points to somewhere in valid user application flash area.
 * Stack pointer at IVT[1] points to somewhere in RAM (0x20000000).

Optonally (untested):

 * The firmware contains its size at offset 0x20 (as a LE uint32).
 * The firmware 32bit XOR checksum is zero (can use offset 0x1C for that).


## Building

First, go to the root /bootloader 
folder, then:

For 32F103...

    make MCU_FAMILY=f103c8 upload
    
Check if you were successful with:

    sudo lsusb

You should see something in lsusb 
from 0x1337:0xC0DE. 

For 32F405/7 family cpus, use...

    make MCU_FAMILY=f40x upload

The LED should be blinking funny, 
this is intentional and means the 
bootloader was successfully entered.

If you mess up, set the PB2 pin to 
its active state with the jumper and 
this will force the MCU back into 
BIOS mode.

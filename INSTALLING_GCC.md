# Installing GNU Tools For ARM
# (c) Adam Munich, 2021

Unfortuately, the apt-get repos for gnu tools for arm are infrequntly updated.
This means we need to do things the hard way and install gcc-arm-none-eabi by
hand. 

Firstly, you'll want to get a copy of the latest gcc-arm-none-eabi. This is 
as simple as going to a suitable place (say, /usr/local/share) and then 
un-packing the tool. 

	$ cd /usr/local/share
	$ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2
	$ tar -xvf gcc-arm-none-eabi-9-2019-q4-major-x86_64-linux.tar.bz2.tar --verbose 

Now, we will need to add symlinks to /usr/bin so make can find the new compiler

	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-as /usr/bin/arm-none-eabi-as 
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++ 
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc 
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-gdb 
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-ld /usr/bin/arm-none-eabi-ld 
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-nm /usr/bin/arm-none-eabi-nm 
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objcopy /usr/bin/arm-none-eabi-objcopy 
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-objdump /usr/bin/arm-none-eabi-objdump
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-readelf /usr/bin/arm-none-eabi-readelf
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-size /usr/bin/arm-none-eabi-size
	$ ln -s /usr/share/gcc-arm-none-eabi-9-2019-q4-major/bin/arm-none-eabi-strip /usr/bin/arm-none-eabi-strip 

Congrats, GNU tools for arm shoudl now be installed. You can verify with:

	$ arm-none-eabi-gcc -v

And you should see:

	gcc version 9.2.1 20191025 (release) [ARM/arm-9-branch revision 277599] (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 


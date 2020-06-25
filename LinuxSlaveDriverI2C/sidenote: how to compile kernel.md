# HOW TO BUILD AND INSTALL KERNEL FROM SOURCE

#### 1 Download latest kernel source from Kernel.org
Download the latest version using your browser or the command:
> wget https://cdn.kernel.org/pub/linux/kernel/v5.x/linux-5.7.2.tar.xz

#### 2 Extract the file:
Decompress the .xz file you download to get a .tar file:
> xz -d -v linux-5.7.2.tar.xz

#### 3 Extract the file (again):
Decompress the .tar file to get a directory containing all the source files for the kernel:
> tar xvf linux-5.7.2.tar

#### 4 Enter the source directory:
You'll need to be inside this directory to run the next commands:
> cd linux-5.7.2

#### 5 Copy existing configurations to the new kernel:
We'll build the kernel using the same configurations your current kernel is using, and then change only what we need:
> sudo cp -v /boot/config-$(uname -r) .config

#### 6 Make sure you have the toolchain to build the source:
To build the kernel you'll need a compiler (GCC) and lots of libraries:
> sudo apt-get install build-essential libncurses-dev bison flex libssl-dev libelf-dev

#### 7 Change the new kernel configurations
Now let's use a program called *menuconfig* to edit the configuration file we just copied and apply the changes we want

##### a. Open the configuration menu
This command will build and run the configuration utility:
> sudo make menucofig

##### b. Enable I2C slave support in
> Device Drivers -> I2C support -> I2C slave support

##### c. Save it with the name ".config"

#### 8 Build the kernel
This will actually compile the kernel. Beware it may take a very long time to complete.
> sudo make -j $(nproc)

#### 9 Install modules
This command will copy the modules that we built to the */lib* directory:
> sudo make modules_install

#### 10 Install the kernel itself
This will copy the kernel image to the */boot* directory:
> sudo make install

#### 11 Update bootloader
Installing a new kernel version does not remove the old version, so you need to tell the bootloader which version to load:
> sudo update-initramfs -c -k 5.7.2

> sudo update-grub

#### 12 Cleanup
The build process generated many GBs of object files, which you may want to delete after you're done:
> sudo make clean

# Linux I2C Slave Driver

### Intro
Linux's kernel has a framework allowing for modules to register themselves as I2C slaves. However, if you wish to use your graphics card HDMI/VGA I2C lines, it's likely thoses drivers do not implement the slave mode. This driver emulates slave mode by bit-banging the SDA and SCL lines, aiming to be compatible with most hardware.

### Operating modes

#### 0 - WIRE_MODE_MASTER_DEFAULT
Implementes master mode using the kernel's default API: *i2c_master_send* and *i2c_master_recv*. This is mode is similar to using the *i2c-dev* module included (probably) with your kernel.

#### 1 - WIRE_MODE_MASTER_BITBANG
Implements master mode by taking over your adapter's SDA and SCL lines and driving them directly.

#### 2 - WIRE_MODE_SLAVE_REGISTER
Implementes slave mode using the kernel's default API: *i2c_slave_register* function. It requires your adapter to support it, which it probably doesn't.

#### 3 - WIRE_MODE_SLAVE_BITBANG
Implements slave mode by taking over your adapter's SDA and SCL lines and driving them directly.

### How to use it
#### In the terminal
##### Set/get the working mode:
You can use cat/echo on the config file for your adapter:
> sudo echo (mode-number) > /dev/wire/(adapter-number)/config

> cat /dev/wire/(adapter-number)/config

Example:
> sudo echo 0 > /dev/wire/4/config

> cat /dev/wire/4/config


##### Reading/writing to an address:
Each device address is handled by a file matching the address' hex number:
> sudo echo (data) > /dev/wire/(adapter-number)/(address)

> sudo head -c (number of bytes) /dev/wire/(adapter-number)/(address)

Example:
> sudo echo "Hello World" > /dev/wire/4/0x5f

> sudo head -c 6 /dev/wire/5/0x3c

#### In a program
Please include **ONLY "i2c-wire-config.h"** in your code.

Open the adapter's configuration and check the supported modes using *ioctl*, then either write to it or use ioctl again to set the mode:
```c
int config_file = open("/dev/wire/4/config", O_RDWR)

if (ioctl(config_file, WIRE_IOCTL_CONFIG_GET_FLAGS, NULL) & WIRE_SUPPORT_BITBANG)
  ioctl(config_file, WIRE_IOCTL_CONFIG_SET_MODE, WIRE_MODE_SLAVE_BITBANG);
```
Open the address' file and read/write normally
```c
int dev;
char dev_path[256];

sprintf(dev_path, "/dev/wire/%d/0x%02x", adapterNr, addr);

if ((dev = open(dev_path, O_RDWR)) > 0)
{
  write(dev, buffer, sizeof(buffer));
}

close(dev);
```
### Buffers, threads, locks & signals
#### Master modes
Master modes are not buffered, so each read/write will immediately issue a matching message in the I2C bus. No background threads are created and no signals are sent to the user application.

#### Slave modes
A background thread will **lock** your adapter and wait for messages issued by the master. You **cannot initiate** a transaction in slave mode. Reading/writing will happen to a **buffer**.

Because the adapter will be in constant use, any other software that attemps to grab a lock on it may end up on an **endless spin_lock**, which may "soft lock" your CPU. So, if you are using your video-board as an I2C adapter, you may want to avoid any software that checks your monitor resolution via EDID (like Arduino IDE for instance).

If your userland application need to be event-driven (you wish to be notified when there is a concerning read/write message on the I2C bus), you can tell this module what your PID is:

```ioctl(i2c_file, WIRE_IOCTL_SLAVE_SIGNAL_PID, getpid())```

Then register the signals to your handler functions:

```signal(I2C_WIRE_SIGNAL_RECEIVED, sighandler_onReceive)```

```signal(I2C_WIRE_SIGNAL_REQUESTED, sighandler_onRequest)```

When the slave receives data the signal is issued **after** the communication is over.

When the master requests data from the slave, the signal is issued **before** this module starts transmitting it's buffer. This design is to allow the user to fill the data to the buffer on the fly. The module will use **clock stretching** to halt the message for *SLAVE_REQUEST_SLEEP_TIME_JIFFIES* (default value 20) and only then transmit the contents of the *txBuffer*. You can do something like this:


```c
void sighandler_onRequest(void)
{
  ioctl(i2c_file, WIRE_IOCTL_SLAVE_PURGE_TX, 0);

  write(i2c_file, new_data, len);

  ioctl(i2c_file, WIRE_IOCTL_SLAVE_WAKE_UP, 0);
}
```

### Requirements
You must enable *CONFIG_I2C_SLAVE* enabled in your kernel. If you don't, you'll need to change this setting and recompile your kernel. The instructions on how to do this are on a sidenote file in this repo.

#### Master modes
WIRE_MODE_MASTER_DEFAULT will always be available but WIRE_MODE_MASTER_BITBANG is only available if your adapter supports direct I/O (see below).

#### Slave modes
WIRE_MODE_SLAVE_REGISTER will be available if your adapter has this functionality by default. You can use WIRE_MODE_SLAVE_BITBANG as a fallback mode if your driver supports supports direct I/O (see below).

#### Bitbanging modes
Thoses modes will hijack the functions your adapter exposes to directly read or write to SCL/SDA (much like GPIOs).
This driver may implement this by 2 different approaches:

##### Algo bit
If your adapter uses Linux's **i2c_algo_bit** module (check *lsmod | grep i2c_algo_bit*) then this driver will access and use the functions
**setsda, setscl, getsda, getscl, pre_xfer and post_xfer** from **struct i2c_algo_bit_data** exposed by your adapter.
##### Recovery
If your adapter exposes **struct i2c_bus_recovery_info** it will be converted to a **struct i2c_algo_bit_data** and the approach described above will be used.

### How to:
##### Build the module:

> make

##### Check the log/output:

> sudo dmesg | grep i2c

##### If you want to run it once but not install it:

> sudo make load

> sudo make unload

##### If you want to install it so it it's loaded at boot every time:

> sudo make install

> sudo make uninstall

### Changelog:
**Jun 2020 -** Project created

### Known bugs and issues:
...

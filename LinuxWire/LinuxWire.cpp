// =============================================================================
// Copyright (C) 2020 Luiz Gustavo Pfitscher e Feldmann
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
// =============================================================================


#include "LinuxWire.h"
#include <stdint.h>     // int types
//#include <stdexcept>    // exceptions
#include <stdio.h>      // printf
#include <errno.h>      // errno variable
#include <string.h>     // strerror function
#include <sys/ioctl.h>  // ioctl function
#include <signal.h>     // to register sigal handlers
#include <fcntl.h>      // file open
#include <unistd.h>     // read/write functions
#include <stdarg.h>     // variadic macros for formatted write

#define CHECK_ADDR(address) if (address > 0x7f) { fprintf(stderr, "\nInvalid I2C slave address\n"); return -ENXIO; }
#define CHECK_MODE(reqmode) if (mode != reqmode){ fprintf(stderr, "\nThis is a %s-mode-only operation\n", (reqmode == MODE_SLAVE) ? "slave" : "master"); return -EINVAL; }

// Initialize Class Variables
void (*LinuxWire::cb_onRequest)(void);
void (*LinuxWire::cb_onReceive)(int);

// Constructors //
LinuxWire::LinuxWire(uint8_t adap)
{
  adapterNr = adap;
  addr = 0;
  i2c_file = -1;
  masterAvailable = 0;

} // actual initialization happens in begin()

LinuxWire::~LinuxWire()
{
  if (i2c_file > 0)
    close(i2c_file);
}

// Public Methods //

int LinuxWire::begin(uint8_t address)
{
    CHECK_ADDR(address);

    addr = address;

    if (addr == 0)
    {
      // we are in master mode
      mode = MODE_MASTER;
    }
    else
    { // we are in slave mode
      int retval;

      mode = MODE_SLAVE;

      if ((retval = configAdapterMode(WIRE_MODE_SLAVE_REGISTER, WIRE_MODE_SLAVE_BITBANG)) != 0)
      {
        bail:
        close(i2c_file);
        i2c_file = -1;
        return retval;
      }

      if ((retval = openI2CaddrDev(addr)) <= 0)
        goto bail;
      else
        i2c_file = retval;

      if ((retval = registerSignalHandlers()) != 0)
        goto bail;
    }

    return 0;
}

int LinuxWire::begin(int address) { return begin((uint8_t)address); }
int LinuxWire::begin(void) { return begin(0); }

int LinuxWire::configAdapterMode(enum wire_mode newmode, enum wire_mode fallbackmode)
{
  int config_file;
  char config_path[256];

  sprintf(config_path, "/dev/wire/%d/config", adapterNr);

  if ((config_file = open(config_path, O_RDWR)) < 0)
  {
    fprintf(stderr, "\nError opening %s configuration file: %d %s\n", config_path, errno, strerror(errno));
    return -errno;
  }

  // the following code writes to the device to set it's mode (equivalent to using "echo *newmode* > /dev/wire/*adapterNr*/config")
  /*char param[3];
  sprintf(param, "%d", newmode);

  if ( (::write(config_file, param, strlen(param))) <= 0)
  {
    fprintf(stderr, "\nError writing value %s to %s: %d %s\n", param, config_path, errno, strerror(errno));
    close(config_file);
    return -errno;
  }*/

  // the following implementation gets the flags from the adapter to see which modes are available
  /*int flags;

  if ((flags = ioctl(config_file, WIRE_IOCTL_CONFIG_GET_FLAGS, NULL)) < 0)
  {
    fprintf(stderr, "\nError reading flags from %s: %d\n", config_path, flags);
    close(config_file);
    return flags;
  }*/

  // the following implementation tries the main mode and then the fallbackmode in case of error
  if (ioctl(config_file, WIRE_IOCTL_CONFIG_SET_MODE, (int)newmode))
  {
    fprintf(stderr, "\n%s: Error: desired mode (%d) not supported - try fallback mode (%d)\n", config_path, newmode, fallbackmode);
    if (ioctl(config_file, WIRE_IOCTL_CONFIG_SET_MODE, (int)fallbackmode))
    {
      fprintf(stderr, "\n%s: Error: desired mode %d not supported (%d)\n", config_path, fallbackmode);
      close(config_file);
      return -1;
    }
  }

  close(config_file); // ok
  return 0;
}

int LinuxWire::openI2CaddrDev(uint8_t addr)
{
  int dev;
  char dev_path[256];

  sprintf(dev_path, "/dev/wire/%d/0x%02x", adapterNr, addr);

  if ((dev = open(dev_path, O_RDWR)) < 0)
  {
    fprintf(stderr, "\nError opening %s I2C device file: %d %s\n", dev_path, errno, strerror(errno));
    return -errno;
  }

  return dev;
}

int LinuxWire::registerSignalHandlers()
{
  int retval;

  if (i2c_file <= 0)
    return -ENOENT;

  if (signal(I2C_WIRE_SIGNAL_RECEIVED, sighandler_onReceive) == SIG_ERR || signal(I2C_WIRE_SIGNAL_REQUESTED, sighandler_onRequest) == SIG_ERR)
  {
      fprintf(stderr, "\nCan't catch request/receive signals!\n");
      return -EIO;
  }

  if ( (retval = ioctl(i2c_file, WIRE_IOCTL_SLAVE_SIGNAL_PID, getpid())) != 0 )
  {
        fprintf(stderr, "\nIoctl failed to register pid to slave %d\n", retval);
        return retval;
  }

  return 0;
}

int LinuxWire::requestFrom(uint8_t address, size_t size)
{
  int retval = 0;

  CHECK_ADDR(address);
  CHECK_MODE(MODE_MASTER);

  if ((retval = beginTransmission(address)) != 0)
    return retval;

  if ((retval = ::read(i2c_file, masterBuffer, size)) > 0)
    masterAvailable = retval;

  if ((retval = endTransmission()) != 0)
    return retval;

  return masterAvailable;
}

int LinuxWire::requestFrom(uint8_t address, uint8_t quantity)                     { return requestFrom(address, static_cast<size_t>(quantity)); }
int LinuxWire::requestFrom(int address, int quantity)                             { return requestFrom(static_cast<uint8_t>(address), static_cast<size_t>(quantity)); }

int LinuxWire::beginTransmission(uint8_t address)
{
  int retval;

  // we must have a valid address and be in master mode to be able to initiate communication
  CHECK_ADDR(address);
  CHECK_MODE(MODE_MASTER);

  // file must be closed before we start transmission
  if (i2c_file > 0)
  {
    fprintf(stderr, "\nAttemp to start transmission while device file already open\n");
    return -EBUSY;
  }

  // set the adapter to master mode
  if ((retval = configAdapterMode(WIRE_MODE_MASTER_DEFAULT, WIRE_MODE_MASTER_BITBANG)) != 0)
  {
    bail:
    close(i2c_file);
    i2c_file = -1;
    return retval;
  }

  // open the addres in master mode
  if ((retval = openI2CaddrDev(address)) <= 0)
    goto bail;
  else
    i2c_file = retval;

  return 0;
}

int LinuxWire::beginTransmission(int address) { return beginTransmission((uint8_t)address); }

int LinuxWire::endTransmission(void)
{
    CHECK_MODE(MODE_MASTER);

    // file must be opened before we close it
    if (i2c_file <= 0)
    {
      fprintf(stderr, "\nAttemp to end a transmission that has not been started\n");
      return -EBADFD;
    }

    close(i2c_file);
    i2c_file = -1;

    return 0;
}

int LinuxWire::write(const uint8_t *data, size_t quantity)
{
  int retval;

  if (i2c_file <= 0)
  {
    if (mode == MODE_SLAVE)
      fprintf(stderr, "\nThe device was not properly opened for writing\n");
    else
      fprintf(stderr, "\nThe master must beginTransmission before writing\n");

    return -EBADFD;
  }

  if ((retval = ::write(i2c_file, (void*)data, quantity)) < 0)
    return -errno;

  return retval;
}

int LinuxWire::read(uint8_t *data, size_t quantity)
{
  if (mode == MODE_SLAVE)
  {
    int retval;

    if (i2c_file <= 0)
    {
      fprintf(stderr, "\nThe device was not properly opened for reading\n");

      return -EBADFD;
    }

    if ((retval = ::read(i2c_file, (void*)data, quantity)) < 0)
      return -errno;

    return retval;
  }
  else
  {
    if (quantity > masterAvailable)
      quantity = masterAvailable;

    memcpy(data, masterBuffer, quantity);
    memmove(masterBuffer, &masterBuffer[quantity], masterAvailable - quantity);
    masterAvailable -= quantity;

    return masterAvailable;
  }
}

int LinuxWire::read(void)
{
  uint8_t data;
  int retval;

  if ((retval = read(&data, 1)) < 0)
    return retval;

  return data;
}

int LinuxWire::write(const char *format, ...)
{
  char buffer[256];

  va_list args;
  va_start (args, format);
  size_t len = vsprintf (buffer,format,args);
  va_end(args);

  return write((uint8_t*)buffer, len);
}

int LinuxWire::write(uint8_t data)
{
  return write(&data, 1);
}

int LinuxWire::flush(void)
{
  if (mode == MODE_SLAVE)
  {
    int retval;

    if (i2c_file <= 0)
      return -EBADF;

    if ( (retval = ioctl(i2c_file, WIRE_IOCTL_SLAVE_PURGE_RX, 0)) != 0)
    {
      fprintf(stderr, "\nFailed to flush RX buffer: %d %d", retval, errno);
      return retval;
    }

    if ( (retval = ioctl(i2c_file, WIRE_IOCTL_SLAVE_PURGE_TX, 0)) != 0)
    {
      fprintf(stderr, "\nFailed to flush TX buffer: %d %d", retval, errno);
      return retval;
    }
  }
  else
    masterAvailable = 0;

  return 0;
}

int LinuxWire::available(void)
{
  if (mode == MODE_SLAVE)
  {
    int retval;

    if (i2c_file <= 0)
      return -EBADF;

    if ( (retval = ioctl(i2c_file, WIRE_IOCTL_SLAVE_POLL_RX, 0)) < 0)
    {
      fprintf(stderr, "\nFailed poll RX buffer: %d %d", retval, errno);
      return retval;
    }

    return retval;
  }
  else
    return masterAvailable;
}

void LinuxWire::sighandler_onReceive(int signal)
{
  if (Wire.mode != MODE_SLAVE)
    return;

  if (!Wire.cb_onReceive)
    return;

  Wire.cb_onReceive(Wire.available());
}

void LinuxWire::sighandler_onRequest(int signal)
{
  if (Wire.mode != MODE_SLAVE)
    return;

  if (!Wire.cb_onRequest)
    return;

  Wire.cb_onRequest();
}

int LinuxWire::onReceive(void (*function)(int))
{
  CHECK_MODE(MODE_SLAVE);
  cb_onReceive = function;

  return 0;
}

int LinuxWire::onRequest(void (*function)(void))
{
  CHECK_MODE(MODE_SLAVE);
  cb_onRequest = function;

  ioctl(i2c_file, WIRE_IOCTL_SLAVE_WAKE_UP, 0);

  return 0;
}


#ifndef WIRE_DEFAULT_ADAPTER
  #define WIRE_DEFAULT_ADAPTER 4
#endif

LinuxWire Wire(WIRE_DEFAULT_ADAPTER);

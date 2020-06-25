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

/// ************************************************* ///
/// please do not include this file in userland apps
/// ************************************************* ///

#ifndef _I2C_WIRE_PRIVATE_H_
#define _I2C_WIRE_PRIVATE_H_

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/i2c.h>

#include "i2c-bit-banging.h"
#include "i2c-wire-config.h"

#define I2C_NUM_ADDRESSES 128
#define READ_WRITE_MAX_COUNT 32
#define CHARDEV_REGION_COUNT (I2C_NUM_ADDRESSES*MAX_DEVICES_COUNT)
#define FILE_GET_WIRE(file_ptr, openfile) wire_open_file* openfile = ((wire_open_file*)(file_ptr)->private_data)

typedef struct wire_device {
        // DEVICE STUFF
        struct device *config_device;
        struct cdev char_dev;
        struct i2c_adapter * adap;
        struct device *sub_device[I2C_NUM_ADDRESSES];

        enum wire_mode mode;
        enum wire_flags flags;

        // FILE ACCESS STUFF
        uint8_t opened_addr[I2C_NUM_ADDRESSES];

        // BUFFER STUFF
} wire_device;

typedef struct wire_open_file {
  int8_t addr;
  wire_device* wiredev;
  void* internal_data;
} wire_open_file;

int i2c_wire_operation(wire_open_file* file, char* buffer, size_t count, uint8_t bRead);
int i2c_wire_open(wire_open_file* file);
int i2c_wire_release(wire_open_file* file);
long i2c_wire_ioctl(wire_open_file* file, unsigned int ioctl_num, unsigned long ioctl_param);

#endif

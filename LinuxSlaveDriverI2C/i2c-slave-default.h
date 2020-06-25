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

#ifndef _I2C_SLAVE_DEFAULT_
#define _I2C_SLAVE_DEFAULT_

#include "i2c-bit-banging.h" // borrows "struct i2c_slave_instance"

struct i2c_slave_instance* i2c_slave_default_instantiate(struct i2c_adapter* i2c, struct i2c_algo_bit_data *algo_data, uint8_t my_addr);
void i2c_slave_default_destroy(struct i2c_adapter* i2c_adap, struct i2c_slave_instance* slave);

int i2c_slave_default_operation(struct i2c_slave_instance* instance, uint8_t bRead, unsigned char *buffer, int count);
long i2c_slave_default_ioctl(struct i2c_slave_instance* slave, unsigned int ioctl_num, unsigned long ioctl_param);

#endif

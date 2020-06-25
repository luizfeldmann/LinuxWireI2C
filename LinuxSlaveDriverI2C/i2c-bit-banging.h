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

#ifndef _I2C_BIT_BANGING_
#define _I2C_BIT_BANGING_

#include <linux/i2c-algo-bit.h>
#include <linux/kthread.h>

/// ============================================================================
/// MASTER MODE FUNCTIONS
/// ============================================================================
int i2c_master_bitbang_operation(struct i2c_adapter* i2c, struct i2c_algo_bit_data *algo_data, uint8_t addr, uint8_t bRead, unsigned char *buffer, int count);

/// ============================================================================
/// SLAVE MODE FUNCTIONS
/// ============================================================================

#define I2C_SLAVE_BUFFER_MAX_SIZE 256

// represents a single slave
typedef struct i2c_slave_instance {
  uint8_t addr;

  spinlock_t buffer_lock;
  volatile uint8_t rxPosition;
  uint8_t rxBuffer[I2C_SLAVE_BUFFER_MAX_SIZE];
  volatile uint8_t txPosition;
  uint8_t txBuffer[I2C_SLAVE_BUFFER_MAX_SIZE];

  int owner_pid;
  volatile int wait_condition;
  wait_queue_head_t wait_queue;
} i2c_slave_instance;

// a thread that listend to events on the I2C bus for a specific adapter
// many slaves may be registered to be treated on the same adapter/thread
typedef struct i2c_slave_thread {
  struct task_struct *kthread;
  struct i2c_adapter* adapter;
  struct i2c_algo_bit_data *algo_data;

  uint8_t numSlaves;
  struct i2c_slave_instance *slaves[128];
} i2c_slave_thread;

// handles allocation, freeing and threading of slaves
struct i2c_slave_instance* i2c_slave_bitbang_instantiate(struct i2c_adapter* i2c, struct i2c_algo_bit_data *algo_data, uint8_t my_addr);
void i2c_slave_bitbang_destroy(struct i2c_adapter* i2c_adap, struct i2c_slave_instance* slave);

// read/write operation to a kernel-space buffer
int i2c_slave_bitbang_operation(struct i2c_slave_instance* instance, uint8_t bRead, unsigned char *buffer, int count);

// allows to poll and flush buffers and more
long i2c_slave_bitbang_ioctl(struct i2c_slave_instance* slave, unsigned int ioctl_num, unsigned long ioctl_param);

// sends a signal to the application that opened the slave address to inform of reads or writes
int i2c_slave_signal_to_owner(struct i2c_slave_instance* slave, int signal);

#endif

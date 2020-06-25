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

#include "i2c-slave-default.h"

#define PRINT_PRELUDE "i2c-slave: "

#define debug_print(format, args...) printk( KERN_NOTICE PRINT_PRELUDE format, args)

typedef struct i2c_slave_container
{
  struct i2c_slave_instance slave;
  struct i2c_client client;
} i2c_slave_container;

int my_i2c_slave_cb(struct i2c_client *cli, enum i2c_slave_event event, u8 *val)
{
  struct i2c_slave_container* container = container_of(cli, struct i2c_slave_container, client);

  switch (event)
  {
    case I2C_SLAVE_WRITE_REQUESTED:
    break;

    case I2C_SLAVE_WRITE_RECEIVED:
      if(container->slave.rxPosition < I2C_SLAVE_BUFFER_MAX_SIZE)
      {
        spin_lock(&container->slave.buffer_lock);
        container->slave.rxBuffer[container->slave.rxPosition++] = *val;
        spin_unlock(&container->slave.buffer_lock);
        debug_print("slave 0x%02x recv: 0x%02x ('%c')\n", container->slave.addr, *val, (char)*val);
      }
      else
        debug_print("slave 0x%02x buffer full! byte 0x%02x ('%c') discarded\n", container->slave.addr, *val, (char)*val);
    break;

    case I2C_SLAVE_READ_PROCESSED:
      spin_lock(&container->slave.buffer_lock);
      memmove(container->slave.txBuffer, &container->slave.txBuffer[1], container->slave.txPosition - 1);
  		container->slave.txPosition -= 1;
      spin_unlock(&container->slave.buffer_lock);
      // fallthrough
    case I2C_SLAVE_READ_REQUESTED:
      spin_lock(&container->slave.buffer_lock);
      *val = container->slave.txBuffer[container->slave.txPosition];
      spin_unlock(&container->slave.buffer_lock);
      // TODO: send signal
    break;

    case I2C_SLAVE_STOP:
      // TODO: send signal
    break;

    default:
    break;
  }

  return 0;
}

int i2c_slave_default_operation(struct i2c_slave_instance* slave, uint8_t bRead, unsigned char *buffer, int count)
{
  return i2c_slave_bitbang_operation(slave, bRead, buffer, count);  // same implementation
}

struct i2c_slave_instance* i2c_slave_default_instantiate(struct i2c_adapter* i2c, struct i2c_algo_bit_data *algo_data, uint8_t my_addr)
{
  int retval;
  struct i2c_slave_container* container;

	// sanity check
	if (i2c == NULL || algo_data == NULL)
		return NULL;

	// create a struct that represents a slave
	container = (struct i2c_slave_container*) kzalloc(sizeof(struct i2c_slave_container), GFP_KERNEL);
	if (container == NULL)
		return NULL;

  container->client = (struct i2c_client) {
  	     .flags = I2C_CLIENT_SLAVE,
      	 .addr = my_addr,
  	     .name = "i2c-wire-slave",
      	 .adapter = i2c,
         //.dev = &wire->device,
         .irq = 0,
         .init_irq = 0,
  };

  if ((retval = i2c_slave_register(&container->client, my_i2c_slave_cb)) != 0)
  {
    debug_print("error registering slave to adapter: %d\n", retval);
    kfree(container);
		return NULL;
  }

	// configures the slave
	container->slave.addr = my_addr;
	container->slave.rxPosition = 0;
  container->slave.txPosition = 0;
	spin_lock_init(&container->slave.buffer_lock);  // this lock prevents the slave thread and the read/write system calls to use the buffer at the same time
  init_waitqueue_head(&container->slave.wait_queue);

  debug_print("slave address 0x%02x registered to adapter %d\n", container->slave.addr, i2c->nr);

  return &container->slave;
}

void i2c_slave_default_destroy(struct i2c_adapter* i2c_adap, struct i2c_slave_instance* remSlave)
{
  struct i2c_slave_container* container = container_of(remSlave, struct i2c_slave_container, slave);

  debug_print("destroy slave listening at addr 0x%02x", container->slave.addr);

  i2c_slave_unregister(&container->client);
	kfree(container);
}

long i2c_slave_default_ioctl(struct i2c_slave_instance* slave, unsigned int ioctl_num, unsigned long ioctl_param)
{
  return i2c_slave_bitbang_ioctl(slave, ioctl_num, ioctl_param); // same implementation
}

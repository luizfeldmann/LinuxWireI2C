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

#include "i2c-wire.h"
#include "i2c-bit-banging.h"
#include "i2c-slave-default.h"

#define OPERATION_FUNCTION_ARGS wire_open_file* file, char* buffer, size_t count, uint8_t bRead
#define OPERATION_FUNCTION_PASS_ARGS file, buffer, count, bRead

// uses linux's default i2c master framework to read or write i2c messages
int i2c_delegate_master_default_operation(OPERATION_FUNCTION_ARGS)
{
  int retval;
  if (bRead)
  {
    int ctl_single_byte_msg = 1;
    size_t count_messages = (ctl_single_byte_msg) ? count : 1;
    size_t msg_index;
    size_t num_bytes_read = 0;

    for (msg_index = 0; msg_index < count_messages; msg_index++)
    {
      struct i2c_msg msg = {
                            .addr = (uint8_t)file->addr,
                            .flags = I2C_M_RD,
                            .len = count / count_messages,
                            .buf = &buffer[msg_index],
                            };

      if (i2c_transfer(file->wiredev->adap, &msg, 1) == 1)
        num_bytes_read += msg.len;
      else
        break; // stop if error
    }

    retval = num_bytes_read;
  }
  else
  {
    struct i2c_msg msg = {
		.addr = (uint8_t)file->addr,
		.flags = 0,
		.len = count,
		.buf = buffer,
    };

	  if ((retval = i2c_transfer(file->wiredev->adap, &msg, 1)) == 1)
    {
      retval = count; // i2c_transfer returned 1 means 1 msg transfered ... fops_write retuns number of bytes written
    }
  }

  return retval;
}

// converts a "bus_recovery_info" into a "i2c_algo_bit_data"
// in other words, uses the adapter's exposed access to i2c "GPIO" that was originally supposed to allow the kernel to recovery weird i2c states
struct i2c_algo_bit_data create_algo_bit_data_from_recovery_data(struct i2c_adapter *adap, struct i2c_bus_recovery_info* info)
{
  void my_setsda(void *data, int state)
  {
    info->set_sda(adap, state);
  }
	void my_setscl(void *data, int state)
  {
    info->set_scl(adap, state);
  }

	int my_getsda(void *data)
  {
    return info->get_sda(adap);
  }

	int my_getscl(void *data)
  {
    return info->get_scl(adap);
  }

  int my_prexfer(struct i2c_adapter* i2c_adap)
  {
    info->prepare_recovery(i2c_adap);
    return 0;
  }

  void my_postxfer(struct i2c_adapter* i2c_adap)
  {
    info->unprepare_recovery(i2c_adap);
  }

  struct i2c_algo_bit_data algo_data = (struct i2c_algo_bit_data) {
            .data = NULL, /// TODO: figure out a way to get this parameter - but is it really necessary, tho ? quite possible not
            .setsda = my_setsda,
            .setscl = my_setscl,
            .getsda = my_getsda,
            .getscl = my_getscl,
            .pre_xfer = my_prexfer,
            .post_xfer = my_postxfer,
            .udelay = 50,
            .timeout = 2,
            .can_do_atomic = 0
          };

  return algo_data;
}


// i2c_algo_bit_data is used by the custom bit-banging algorithm to issue i2c messages
int i2c_delegate_master_recovery_operation(OPERATION_FUNCTION_ARGS)
{
  struct i2c_algo_bit_data algo_data = create_algo_bit_data_from_recovery_data(file->wiredev->adap, file->wiredev->adap->bus_recovery_info);

  return i2c_master_bitbang_operation(file->wiredev->adap, &algo_data, file->addr, bRead, (unsigned char*)buffer, count);
}

// if the adapter exposed to linux some functions to access the I2C lines' GPIOs ...
// (which are normally used by the "i2c_algo_bit" module of the kernel)
// then whe can relay those functions to our own (using bit-banging) implementation of i2c
int i2c_delegate_master_bitbang_operation(OPERATION_FUNCTION_ARGS)
{
  // should we send all the buffer in a single message or in single-byte-messages?
  int ctl_single_byte_msg = bRead ? 1 : 0;
  // for now, let's do 1 byte a time on reads, multiple bytes on write
  // because Wire::onReceive(howMany) has the number of bytes as argument
  // but Wire::onRequest has no length parameter ...
  // I *think* singularizing the requests may increase chances of returning success on at least some of the operations

  size_t count_messages = (ctl_single_byte_msg) ? count : 1;  // how many messages we'll process
  size_t msg_index;                                           // current message index
  size_t num_bytes = 0;                                       // count total processed bytes
  int msg_len = count / count_messages;                       // size of each message; will be either "count" or 1
  int retval;                                                 // return value of the actual function call

  // do all the messages (or maybe only one ...)
  for (msg_index = 0; msg_index < count_messages; msg_index++)
  {
    if (( retval = i2c_master_bitbang_operation(file->wiredev->adap, (struct i2c_algo_bit_data *)file->wiredev->adap->algo_data,
    (uint8_t)file->addr, bRead, (unsigned char*)&buffer[msg_index], msg_len)) == msg_len )
      num_bytes += msg_len;
    else
      return retval;
  }

	return num_bytes; // we went all the way without any errors :)
}

// if the adapter complies to linux's default "slave framework", then we can use it as it was originally intended ...
int i2c_delegate_slave_register_operation(OPERATION_FUNCTION_ARGS)
{
  /*int retval = 0;

  struct i2c_client client = (struct i2c_client) {
	     .flags = I2C_CLIENT_SLAVE,
    	 .addr = wire->opened,
	     .name = "i2c-wire-slave",
    	 .adapter = wire->adap,
       //.dev = &wire->device,
       .irq = 0,
       .init_irq = 0,
  };

  int my_i2c_slave_cb(struct i2c_client *cli, enum i2c_slave_event event, u8 *val)
  {
    switch (event)
    {
      case I2C_SLAVE_WRITE_RECEIVED:
       break;
      case I2C_SLAVE_READ_PROCESSED:
        break;
      case I2C_SLAVE_READ_REQUESTED:
       break;
      case I2C_SLAVE_STOP:
       break;
      case I2C_SLAVE_WRITE_REQUESTED:
      break;
      default:
      break;
    }

    return 0;
  }

  retval = i2c_slave_register(&client, my_i2c_slave_cb);
	if (retval)
		return retval;

  // ** TODO ** // keep waiting for semaphore or something

  i2c_slave_unregister(&client);

  // ** TODO ** // process the results ...*/

  return -ENOSYS; // not implemented yet ....
}


// this functions executes an operation using the appropriate delegate function depending on the configured operation mode of the wire
int i2c_wire_operation(OPERATION_FUNCTION_ARGS)
{
  switch (file->wiredev->mode)
  {
    case WIRE_MODE_MASTER_DEFAULT:
      return i2c_delegate_master_default_operation(OPERATION_FUNCTION_PASS_ARGS);
      break;

    case WIRE_MODE_SLAVE_BITBANG:
      // here we use our own implementation of a bit-banging algorithm and relay the pin read/write logic to the functions exposed by the adapter
      return i2c_slave_bitbang_operation((struct i2c_slave_instance*)file->internal_data, bRead, buffer, count);
      break;

    case WIRE_MODE_MASTER_BITBANG:
      // implements the bitbanging directly by the algorithm data
      if (file->wiredev->flags & WIRE_SUPPORT_BITBANG)
        return i2c_delegate_master_bitbang_operation(OPERATION_FUNCTION_PASS_ARGS);
      // implements the bitbanging by converting the recovery functions
      else if (file->wiredev->flags & WIRE_SUPPORT_RECOVERY)
        return i2c_delegate_master_recovery_operation(OPERATION_FUNCTION_PASS_ARGS);
      else
        return -ENOSYS;
      break;

    case WIRE_MODE_SLAVE_REGISTER:
      return i2c_delegate_slave_register_operation(OPERATION_FUNCTION_PASS_ARGS);
      break;

    default:
      return -ENOSYS; // not implemented
      break;
  }
}

int i2c_wire_openrelease(wire_open_file* file, uint8_t open)
{
  switch (file->wiredev->mode)
  {
    case WIRE_MODE_MASTER_DEFAULT:
    case WIRE_MODE_MASTER_BITBANG:                                              // these modes require no special preparations
      return 0;
    break;

    case WIRE_MODE_SLAVE_BITBANG:                                               // this mode requires creating a buffer and a kthread to read the adapater in a busy loop
      if (open)
      {
        if (file->wiredev->flags & WIRE_SUPPORT_BITBANG)                        // implements the bitbanging directly by the algorithm data
        {
          if ((file->internal_data = i2c_slave_bitbang_instantiate(file->wiredev->adap, (struct i2c_algo_bit_data *)file->wiredev->adap->algo_data, (uint8_t)file->addr)) == NULL)
            return -EIO;
          else
            return 0;
        }
        else if (file->wiredev->flags & WIRE_SUPPORT_RECOVERY)                  // implements the bitbanging by converting the recovery functions
        {
          /// TODO ///
          return -ENOSYS;
        }
        else
          return -ENOSYS;
      }
      else
      {
        if (file->internal_data == NULL)
          return -EINVAL;

        i2c_slave_bitbang_destroy(file->wiredev->adap, (struct i2c_slave_instance*)file->internal_data);

        return 0;
      }

    break;

    case WIRE_MODE_SLAVE_REGISTER:                                              // this mode will require registering/unregistring callbacks and creation of a buffer
      return 0;
    break;

    default:
      return -ENOSYS;                                                           // not implemented
    break;
  }
}

int i2c_wire_open(wire_open_file* file) { return i2c_wire_openrelease(file, 1); }
int i2c_wire_release(wire_open_file* file) { return i2c_wire_openrelease(file, 0); }

long i2c_wire_ioctl(wire_open_file* file, unsigned int ioctl_num, unsigned long ioctl_param)
{
  switch (file->wiredev->mode)
  {
    case WIRE_MODE_SLAVE_BITBANG:
      return i2c_slave_bitbang_ioctl((struct i2c_slave_instance*)file->internal_data, ioctl_num, ioctl_param);
    break;

    case WIRE_MODE_SLAVE_REGISTER:
      return -ENOSYS; // TODO: implement this
    break;

    case WIRE_MODE_MASTER_DEFAULT: // this mode implements no IOCTL
    case WIRE_MODE_MASTER_BITBANG: // this mode implements no IOCTL
    default:
      return -EBADR; // bad request
    break;
  }
}

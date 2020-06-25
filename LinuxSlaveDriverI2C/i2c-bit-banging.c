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

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include "i2c-bit-banging.h"


#define PRINT_PRELUDE "i2c-bit-bang: "

#define DEBUG

#ifdef DEBUG
	#define debug_print(format, args...) printk( KERN_NOTICE PRINT_PRELUDE format, args)
#else
	#define debug_print(format, args...) (void)0;
#endif
/// ============================================================================
/// UTILITY FUNCTIONS
/// ============================================================================

// NOTICE: SOME OF THIS CODE IS BORROWED FROM linux/drivers/i2c/algos/i2c-algo-bit.c (with modifications)
#define setsda(adap, val)	adap->setsda(adap->data, val)
#define setscl(adap, val)	adap->setscl(adap->data, val)
#define getsda(adap)		adap->getsda(adap->data)
#define getscl(adap)		adap->getscl(adap->data)

// sets SDA line to LOW
static inline void sdalo_delay(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 0);
	udelay((adap->udelay + 1) / 2);
}

// sets SDA line to HIGH
static inline void sdahi_delay(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 1);
	udelay((adap->udelay + 1) / 2);
}

// sets SCL line to LOW
static inline void scllo_delay(struct i2c_algo_bit_data *adap)
{
	setscl(adap, 0);
	udelay(adap->udelay / 2);
}

// sets SCL line to HIGH and waits until it's actually set to HIGH
static int sclhi_delay(struct i2c_algo_bit_data *adap)
{
	unsigned long start;

	setscl(adap, 1); // sets the clock line HI

	if (!adap->getscl) // if there is no registered function to sense the actual state of the line, this function returns ok always
		goto done;

	// the SCL line my take a while to go HIGH because the slave may be slow and will pursposefully hold the line to gain time to process it's data
	// this is called "slave clock stretching" and requires the master to hold on for a while
	// but holding for too long will block the CPU on a "soft lock" so we MUST implement some timeout

	start = jiffies;
	while (!getscl(adap))
	{
		if (time_after(jiffies, start + adap->timeout)) // make sure we don't wait too long and cause a "cpu soft lock"
		{
			if (getscl(adap))
				break;
			return -ETIMEDOUT;
		}

		cpu_relax();
	}

done:
	udelay(adap->udelay);
	return 0;
}

/// ============================================================================
/// MASTER MODE FUNCTIONS
/// ============================================================================

// sends a start condition
// SDA and SCL lines are originally HIGH
// SDA falls low while SCL line remais high
// after the SCL line falls low too, the start condition is done and data can be sent
static inline void i2c_master_start_condition(struct i2c_algo_bit_data *adap)
{
	setsda(adap, 0);
	udelay(adap->udelay);
	scllo_delay(adap);
	udelay(adap->udelay);
}

// sends a stop condition
// SDA line is originall low
// then, the SDA rises while SCL is also HIGH
// this signals the communitication is terminated
static inline void i2c_master_stop_condition(struct i2c_algo_bit_data *adap)
{
	sdalo_delay(adap);
	sclhi_delay(adap);
	setsda(adap, 1);
	udelay(adap->udelay);
}

// returns
// 0 - ok
// <0 - err
static int i2c_master_write_bit(struct i2c_algo_bit_data *adap, uint8_t bit)
{
	int retval = 0;

	// set the appropriate vale to the SDA line
  if(bit == 0)
    sdalo_delay(adap);
  else
    sdahi_delay(adap);

	udelay((adap->udelay + 1) / 2);

  if ((retval = sclhi_delay(adap)) != 0)
		return retval;

	udelay((adap->udelay + 1) / 2);

  scllo_delay(adap);

	return retval;
}


// returns
// 0/1 - bit value
// <0 - err
static int i2c_master_read_bit(struct i2c_algo_bit_data *adap)
{
	int inbit = 0;
	int retval = 0;

  sdahi_delay(adap);            // 'hi' is the same as releasing the line

	if ((retval = sclhi_delay(adap)) != 0)
		return retval;

	udelay((adap->udelay + 1) / 2);

	inbit = getsda(adap);
	scllo_delay(adap);

	udelay((adap->udelay + 1) / 2);

	return inbit;
}

// returns:
// 0 - nak
// 1 - ack
// <0 - err
static int i2c_master_write_byte(struct i2c_algo_bit_data *adap, uint8_t byte )
{
	int retval = 0;
	uint8_t bit_n = 0;

  for ( bit_n = 0; bit_n < 8; bit_n ++)
  {
    if ( (retval = i2c_master_write_bit(adap, byte & 0x80)) != 0 )
			return retval; // timeout error writing bit

    byte <<= 1;
  }

  return (!i2c_master_read_bit(adap)); // get ack or nack
}

static int i2c_master_read_byte(struct i2c_algo_bit_data *adap)
{
	int retval = 0;
	uint8_t bit_n = 0;
	uint8_t res = 0;

  for(bit_n =0; bit_n < 8; bit_n++)
  {
		if ((retval = i2c_master_read_bit(adap)) < 0)
			return retval; // error reading bit

    res <<= 1;
    res |= (uint8_t)retval;
  }

  if ( (retval = i2c_master_write_bit(adap, 0)) == 0) // send ack
		return res; // ack was sent - return the read value
	else
		return retval; // ack not sent - some error
}

static int i2c_master_read_write_buffer(struct i2c_algo_bit_data *adap, uint8_t *buffer, int count, uint8_t bRead)
{
	int retval;
	int wrcount = 0; // keep track how many bytes where written

	while (count > 0)
	{
		if (bRead)
		{
			retval = i2c_master_read_byte(adap);
			if (retval < 0)
			{
				debug_print("read: TIMEOUT in byte %d\n", wrcount);
				return retval; // error
			}
			else
			{
				*buffer = (uint8_t)retval;
				debug_print("read: ACK in byte %d (0x%02x '%c')\n", wrcount, *buffer, *buffer);
			}
		}
		else
		{
			retval = i2c_master_write_byte(adap, *buffer); // CHECK LATER: should we send ACK for the last byte too ?
			if (retval == 0)
			{
				debug_print("write: NAK in byte %d\n", wrcount);
				return retval;
			}
			else if (retval < 0)
			{
				debug_print("write: TIMEOUT in byte %d\n", wrcount);
				return retval; // error
			}
			else
				debug_print("write: ACK in byte %d\n", wrcount);
		}

		count--;
		buffer++;
		wrcount++;
	}

	return wrcount;
}

// converts a 7-bit device address and a read/write flag into an 8-bit address
static inline uint8_t i2c_8bit_addr(const uint8_t addr, const uint8_t bRead)
{
	return (addr << 1) | (bRead ? 1 : 0);
}

int i2c_master_bitbang_operation(struct i2c_adapter* i2c, struct i2c_algo_bit_data *algo_data, uint8_t addr, uint8_t bRead, unsigned char *buffer, int count)
{
	int retval = -EIO;

	// lock the adapter to prevent inteferences from other modules
	i2c_lock_bus(i2c, I2C_LOCK_SEGMENT);

	// prepare the module to be driven
	if (algo_data->pre_xfer)
		if ((retval = algo_data->pre_xfer(i2c)) < 0)
			goto bailout;

  // actual code
	i2c_master_start_condition(algo_data);
	if ( (retval = i2c_master_write_byte(algo_data, i2c_8bit_addr(addr, bRead))) < 0)
	{
			debug_print("NAK from addr 0x%02x in start of message\n", addr);
			goto premature_stop;
	}
	else
	{
			debug_print("ACK from addr 0x%02x in start of message\n", addr);
			retval = i2c_master_read_write_buffer(algo_data, buffer, count, bRead);
	}

	debug_print("finish %s operation with %d bytes at addr 0x%02x\n", bRead ? "READ" : "WRITE", count, addr);

	premature_stop:
	i2c_master_stop_condition(algo_data);
	// end of actual code

	// tell the adapter we are done
	if (algo_data->post_xfer)
		algo_data->post_xfer(i2c);

	bailout:
	i2c_unlock_bus(i2c, I2C_LOCK_SEGMENT); // release it
	return retval;
}

/// ============================================================================
/// SLAVE MODE FUNCTIONS
/// Slave devices send or receive data upon read-request or write-request by a master
/// Slaves cannot initiate a communitication, only respond when request
/// Slaves must read the line states constantly to determine when there is an ongoing operation
/// The master drives the clock line and, therefore, sets the timing
/// The slave must match the masters timing by reading the SCL line
/// ============================================================================

#include <linux/sched.h>
#include <linux/preempt.h>
#include <asm/siginfo.h>
#include <linux/pid_namespace.h>
#include <linux/pid.h>
#include "i2c-wire-config.h"

const u64 slave_timeout_ns = 100000000;

// the userland application that opened this slave address may register it's PID using IOCTL
// in that case, will send signals to the application when new data is available or when it has been requested
int i2c_slave_signal_to_owner(struct i2c_slave_instance* slave, int signal)
{
	int ret;
	struct task_struct *owner_task;
	struct kernel_siginfo kernsig;

	if (slave->owner_pid <= 0)
		return 0;

  memset(&kernsig, 0, sizeof(struct kernel_siginfo));
	kernsig.si_signo = signal;
	kernsig.si_code = SI_KERNEL;

	rcu_read_lock();
	owner_task = pid_task(find_vpid(slave->owner_pid), PIDTYPE_PID);
	rcu_read_unlock();

	ret = send_sig_info(signal, &kernsig, owner_task);

	debug_print("send signal %d to slave 0x%02x owner pid %d: %d %s\n", signal, slave->addr, slave->owner_pid, ret, (ret < 0) ? "error" : "ok");

	return ret;
}

static int test_bus(struct i2c_adapter *i2c_adap, struct i2c_algo_bit_data *adap)
{
	/// CODE BORROWED FROM linux's drivers/i2c/algos/i2c-algo-bit.c [with modifications]
	/// GPL v2 - please check that file for more information
	int scl, sda, ret;

	// this is called when the adapter is already being used
	// so close it
	if (adap->post_xfer)
		adap->post_xfer(i2c_adap);

	// and then open again
	if (adap->pre_xfer)
	{
		ret = adap->pre_xfer(i2c_adap);
		if (ret < 0)
			return -ENODEV;
	}

	sda = getsda(adap);
	scl = getscl(adap);
	if (!scl || !sda)
	{
		printk(KERN_WARNING "i2c bus seems to be busy (scl=%d, sda=%d)\n", scl, sda);
		goto bailout;
	}

	sdalo_delay(adap);
	sda = getsda(adap);
	scl = getscl(adap);
	if (sda)
	{
		printk(KERN_WARNING "SDA stuck high!\n");
		goto bailout;
	}
	if (!scl)
	{
		printk(KERN_WARNING "SCL unexpected low while pulling SDA low!\n");
		goto bailout;
	}

	sdahi_delay(adap);
	sda = getsda(adap);
	scl = getscl(adap);
	if (!sda)
	{
		printk(KERN_WARNING "SDA stuck low!\n");
		goto bailout;
	}
	if (!scl)
	{
		printk(KERN_WARNING "SCL unexpected low while pulling SDA high!\n");
		goto bailout;
	}

	scllo_delay(adap);
	sda = getsda(adap);
	scl = getscl(adap);
	if (scl)
	{
		printk(KERN_WARNING "SCL stuck high!\n");
		goto bailout;
	}
	if (!sda)
	{
		printk(KERN_WARNING "SDA unexpected low while pulling SCL low!\n");
		goto bailout;
	}

	sclhi_delay(adap);
	sda = getsda(adap);
	scl = getscl(adap);
	if (!scl)
	{
		printk(KERN_WARNING "SCL stuck low!\n");
		goto bailout;
	}
	if (!sda)
	{
		printk(KERN_WARNING "SDA unexpected low while pulling SCL high!\n");
		goto bailout;
	}

	pr_info("Test OK\n");
	return 0;

bailout:
	sdahi_delay(adap);
	sclhi_delay(adap);

	return -ENODEV;
}

// this thread loops reading the I2C bus and buffering incoming messages
// or sending out the contents buffered in txBuffer upon request by master
int i2c_slave_thread_runner(void *data)
{
	// states and events
	enum slave_event
	{
		CLOCK_RISE,
		CLOCK_FALL,
		CLOCK_HIGH,
		CLOCK_LOW,
		START_COND,
		STOP_COND,
	};

	enum slave_state
	{
		STATE_IDLE,
		STATE_READ_ADDR,
		STATE_MASTER_READ,
		STATE_MASTER_WRITE,
	};

	// declarations, initializations
	volatile u64 start_time;																											// timeout timer
	uint8_t sda_old, sda_new, scl_old, scl_new;																		// pin logic states
	enum slave_state current_state = STATE_IDLE;																	// the current operation being done by the thread
	uint8_t bit_number, byte_value, clock, ack;																		// state variables

	struct i2c_slave_instance* slave = NULL;																			// the slave instance that is being communicated to
	struct i2c_slave_thread* thread = (struct i2c_slave_thread*)data;							// represents the adapter attached to this slave thread

	inline void slave_event_startCondition(void)
	{
		// disable preemption
		preempt_disable();

		// first operation is to read the address
		current_state = STATE_READ_ADDR;

		// get the current time to keep track of timeout
		start_time = ktime_get_mono_fast_ns();

		debug_print("START condition detected on adapter %d\n", thread->adapter->nr);
	}

	// reenable preemption
	inline void slave_event_stopCondition(uint8_t bTimeout, uint8_t bail)
	{
		// release our buffer
		if (slave != NULL)
		{
			if (current_state == STATE_MASTER_WRITE) // we signal the user about arrived data AFTER the end of message
				i2c_slave_signal_to_owner(slave, I2C_WIRE_SIGNAL_RECEIVED);

			slave = NULL;
		}

		// reenable preempt to allow cpu to multitask
		preempt_enable();

		// no operation
		current_state = STATE_IDLE;

		if (bTimeout)
		{
			// release the lines so they wont get stuck and hold the master in a "never ending clock stretch"
			test_bus(thread->adapter, thread->algo_data);
		}

		if (!bail)
			debug_print("%s on adapter %d\n", bTimeout ? "TIMEOUT" : "STOP condition", thread->adapter->nr);
	}

	inline uint8_t slave_event_getNextByteToSend(void)
	{
		uint8_t next_byte = 0x00;

		if (slave == NULL)
			return 0;

		if (slave->txPosition)
		{
			spin_lock(&slave->buffer_lock);

			next_byte = slave->txBuffer[0];
			memmove(slave->txBuffer, &slave->txBuffer[1], slave->txPosition - 1);
			slave->txPosition -= 1;

			spin_unlock(&slave->buffer_lock);
		}

		return next_byte;
	}

	inline void event_readRequested(void)
	{
		if (!slave->owner_pid)
			return;

		#ifdef SLAVE_REQUEST_SLEEP_TIME_JIFFIES
		setscl(thread->algo_data, 0);																										// hold the clock low (clock stretching) to halt the communitication
		slave->wait_condition = 0;																											// set 0 so the wait won't end right away
		#endif

		i2c_slave_signal_to_owner(slave, I2C_WIRE_SIGNAL_REQUESTED);										// signal the user to fill our buffer

		#ifdef SLAVE_REQUEST_SLEEP_TIME_JIFFIES
		preempt_enable();																																// let the cpu do other stuff
		wait_event_interruptible_timeout( slave->wait_queue, slave->wait_condition,
																								SLAVE_REQUEST_SLEEP_TIME_JIFFIES);	// wait for user to tell us the buffer is filled or to timeout (in jiffies)
		preempt_disable();																															// monopolyze the cpu to guarantee good communitication
		//////////////////////setscl(thread->algo_data, 1); 														// if we release the clock here, the message will fail - we first need to set SDA and *only then* release the SCL
		start_time = ktime_get_mono_fast_ns();																					// make sure we don't timeout the main loop because the time we spent while comms were halted
		#endif
	}

	inline void slave_event_byteSent(uint8_t sent_byte, uint8_t ack)
	{
		if (slave == NULL)
			return;

		debug_print("slave 0x%02x sent (%s): %u ('%c')\n", slave->addr, ack ? "ACK" : "NACK", (uint8_t)sent_byte, (char)sent_byte);
	}

	inline void slave_event_byteRecv(uint8_t recv_byte)
	{
		if (slave == NULL)
			return;
		if (slave->rxPosition < I2C_SLAVE_BUFFER_MAX_SIZE)
		{
			spin_lock(&slave->buffer_lock);
			slave->rxBuffer[slave->rxPosition++] = recv_byte; // append our byte to the buffer
			spin_unlock(&slave->buffer_lock);

			debug_print("slave 0x%02x recv: 0x%02x ('%c')\n", slave->addr, recv_byte, (char)recv_byte);
		}
		else
			debug_print("slave 0x%02x buffer full! byte 0x%02x ('%c') discarded\n", slave->addr, recv_byte, (char)recv_byte);
	}

	void slave_state_write(enum slave_event event)
	{
		if (slave == NULL)
			return;

		switch (event)
		{
			case START_COND:
				event_readRequested(); // causes a "clock stretching" allowing time for user to fill the buffer
				START:
				bit_number = 0;
				byte_value = slave_event_getNextByteToSend();
				//debug_print("will send %c\n", byte_value);
				// fallthrough
			case CLOCK_FALL: // end of current bit cycle - prepare for next bit
				if (bit_number < 8)
				{
					setsda(thread->algo_data, ((byte_value << bit_number) & 0x80) ? 1 : 0);

					if (bit_number == 0)
						setscl(thread->algo_data, 1); // release the clock stretching

					bit_number++;
				}
				else if (bit_number == 8)
				{
					setsda(thread->algo_data, 1); // release SDA so we can read ack/nack
					bit_number++;
				}
				else if (bit_number == 10)
				{
					slave_event_byteSent(byte_value, ack);

					if (!ack)
						slave_event_stopCondition(0, 1);
					else
						goto START;
				}
			break;

			case CLOCK_RISE:
				if (bit_number == 9)
				{
					ack = !getsda(thread->algo_data);
					bit_number++;
				}
			break;

			default: break;
		}
	}

	void slave_state_read(enum slave_event event)
	{
		switch (event)
		{
			case START_COND:
				bit_number = 0;
				byte_value = 0;
				clock = 0;
				break;

			case CLOCK_RISE:
				if (clock >= 100)
				{
					//debug_print("ack clock rise%s\n", "");
				}
				else
				{
					clock++;
				}
				break;

			case CLOCK_FALL: // write value on fall (end of bit)
				if (clock >= 100)
				{
					//debug_print("ack clock fall %s\n", "release");
					setsda(thread->algo_data, 1); // set high = release
					if (clock == 100)
					{
						current_state = STATE_MASTER_WRITE;
					}
					else
					{
						current_state = STATE_MASTER_READ;
						slave_state_write(START_COND);
					}
					clock = 0;
				}
				else if (clock) // must check there was a rise before, because there is a fall after the start condition
				{
					bit_number++;
					byte_value <<= 1;
					byte_value |= sda_old;
					//debug_print("read fall bit %u = %u\n", bit_number, sda_new);
				}
				break;

			default: break;
		}

		if (bit_number == 8)
		{
			if (current_state == STATE_READ_ADDR)
			{
				uint8_t addr = (byte_value & 0xFE) >> 1;
				uint8_t oper = byte_value & 0x01; // 0 - master write; 1 - master read; 2 - addr read

				slave = thread->slaves[addr];

				debug_print("master %s addr 0x%02x (%s)\n", ((oper) ? "READ from" : "WRITE to"), addr, ((slave) ? "attached" : "not attached"));

				if (slave)
				{
					if (oper)
						goto ACK2; // master read / slave write
					else
						goto ACK1; // master write / slave read
				}
				else
				{
					bit_number = 0;
					byte_value = 0;
					clock = 0;
					slave_event_stopCondition(0, 1); // bail from this message
					return;
				}
			}
			else
			{
				slave_event_byteRecv(byte_value);
				goto ACK1;
			}

			ACK1: clock = 100; goto ACK;
			ACK2: clock = 101; goto ACK;

			ACK:
			setsda(thread->algo_data, 0); // set low = ACK
			//debug_print("ack set%s\n", "");
			bit_number = 0;
			byte_value = 0;
		}
	}

	void state_run(enum slave_event event)
	{
		switch (current_state)
		{
			case STATE_IDLE:
				cpu_relax();
				break;

			case STATE_READ_ADDR:
				slave_state_read(event);
				break;

			case STATE_MASTER_READ:
				slave_state_write(event);
				break;

			case STATE_MASTER_WRITE:
				slave_state_read(event);
				break;
		}
	}

	if (data == NULL)
		return -EINVAL;

	// initiate the pin values
	setsda(thread->algo_data, 1); // set high = release - make sure the line isnt stuck low blocking the bus
	sda_old = getsda(thread->algo_data);
	scl_old = getscl(thread->algo_data);

	// keep reading the lines
	while (!kthread_should_stop())
	{
		// get the new pin values
		REPEAT:
		sda_new = getsda(thread->algo_data);
		scl_new = getscl(thread->algo_data);

		if (scl_old == scl_new) // clock didn't change
		{
			if (scl_new) // it was hi
			{
				if (sda_old && !sda_new) // but sda fell
				{
					slave_event_startCondition();
					state_run(START_COND);
				}
				else if (!sda_old && sda_new) // but sda rose
				{
					state_run(STOP_COND);
					slave_event_stopCondition(0, 0);
				}
				else
					state_run(CLOCK_HIGH);
			}
			else
				state_run(CLOCK_LOW);
		}
		else
		{
			if (scl_new)
				state_run(CLOCK_RISE);
			else
				state_run(CLOCK_FALL);
		}

		// legate the states
		sda_old = sda_new;
		scl_old = scl_new;

		if (current_state == STATE_IDLE)
		{
			// let the kernel work while not busy
			if (should_resched(0))
				schedule();
		}
		else
		{
			// check for timeout
			if (ktime_get_mono_fast_ns() > start_time + slave_timeout_ns)
				slave_event_stopCondition(1, 0);
			else
				goto REPEAT; // do not check for thread exit during operation to keep loop faster - threead wil exit while in idle
		}
	}

	return 0;
}

// handles a read/write system call
// writes: buffers the data to be sent by a separate thread when the master requests data
// read: read the data present in the buffer that was previously saved by the listener thread
// returns: the number of processed bytes or negative error code
int i2c_slave_bitbang_operation(struct i2c_slave_instance* slave, uint8_t bRead, unsigned char *buffer, int count)
{
	int retval;

	if (bRead)
	{
		if (slave->rxPosition == 0)
			retval = 0;
		else
		{
			if (count > slave->rxPosition) // we limit ourselves to read only the available amount of bytes
				count = slave->rxPosition;

			spin_lock(&slave->buffer_lock);

			// copy the I2C buffer to the systemcall's buffer
			memcpy(buffer, slave->rxBuffer, count);

			// move the I2C buffer back to maintain FIFO
			memmove(&slave->rxBuffer[0], &slave->rxBuffer[count], slave->rxPosition - count); // we need to used memmove because it can handle overlapping memory areas - memcpy can't
			slave->rxPosition -= count;

			spin_unlock(&slave->buffer_lock);

			retval = count; // we return the number of bytes
		}
	}
	else
	{
		if (slave->txPosition + count >= I2C_SLAVE_BUFFER_MAX_SIZE - 1) // limit the write size to avoid overflowing the buffer
			count = I2C_SLAVE_BUFFER_MAX_SIZE - 1 - slave->txPosition;

		memcpy(&slave->txBuffer[slave->txPosition], buffer, count);
		slave->txPosition += count;

		retval = count;
	}

	return retval;
}

long i2c_slave_bitbang_ioctl(struct i2c_slave_instance* slave, unsigned int ioctl_num, unsigned long ioctl_param)
{
	switch (ioctl_num)
	{
		// returns the number of bytes in receive buffer
		case WIRE_IOCTL_SLAVE_POLL_RX:
			return slave->rxPosition;
		break;

		// returns number if bytes in transmit buffer
	  case WIRE_IOCTL_SLAVE_POLL_TX:
			return slave->txPosition;
		break;

		// clears the transmit buffer
		case WIRE_IOCTL_SLAVE_PURGE_TX:
			spin_lock(&slave->buffer_lock);
			slave->txPosition = 0;
			spin_unlock(&slave->buffer_lock);
		break;

		// clears the receive buffer
		case WIRE_IOCTL_SLAVE_PURGE_RX:
			spin_lock(&slave->buffer_lock);
			slave->rxPosition = 0;
			spin_unlock(&slave->buffer_lock);
		break;

		// register the userland application PID to receive signals when something happens (receive or transmit data "events")
		case WIRE_IOCTL_SLAVE_SIGNAL_PID:
			slave->owner_pid = (int)ioctl_param;
			debug_print("slave 0x%02x owned by pid %d via IOCTL\n", slave->addr, slave->owner_pid);
		break;

		case WIRE_IOCTL_SLAVE_WAKE_UP:
			slave->wait_condition = 1;
			wake_up_interruptible(&slave->wait_queue);
		break;

		default:
			return -EINVAL; // received some unknown command - return invalid argument error;
		break;
	}

	return 0;
}

static i2c_slave_thread threads[MAX_DEVICES_COUNT];

// called when creating a slave in the i2c_adapter
// if it is the first slave, the adapter must be configured to support it
// this is done by locking the adapter and then calling "pre_xfer", which enables the GPIO access functions
// a thread is created to monitor messages on that adapter
// if the adapter has been configured before, this function only adds the slave to the thread's registry (attaches it)
// returns: zero on success or negative error code
int setupAdapter(struct i2c_adapter* i2c_adapter, struct i2c_algo_bit_data *algo_data, struct i2c_slave_instance* newSlave)
{
	int retval = 0;
	i2c_slave_thread* thread = &threads[i2c_adapter->nr];

	if (thread->numSlaves && thread->kthread)
	{
		addSlave:
		thread->slaves[newSlave->addr] = newSlave;
		thread->numSlaves++;
		debug_print("added slave address 0x%02x to thread '%s' now with %d slaves attached\n", newSlave->addr, thread->kthread->comm, thread->numSlaves);
		return 0;
	}

  thread->adapter = i2c_adapter;
  thread->algo_data = algo_data;
	thread->numSlaves = 0;

	// lock the adapter so only this algo can use it
	i2c_lock_bus(i2c_adapter, I2C_LOCK_SEGMENT);

	// tell the adapter we are going to used it
	if (algo_data->pre_xfer)
	{
		if ((retval = algo_data->pre_xfer(i2c_adapter)) < 0)
		{
			i2c_unlock_bus(i2c_adapter, I2C_LOCK_SEGMENT);

			debug_print("adapter %d failed to lock and prepare for use\n", i2c_adapter->nr);
			return retval;
		}
	}
	else debug_print("adapter %d is locked and assigned to slave mode\n", i2c_adapter->nr);

	thread->kthread = kthread_run(i2c_slave_thread_runner, thread, "i2cSlaveAdap%d", i2c_adapter->nr);
	if (IS_ERR(thread->kthread))
	{
		debug_print("error creating thread%s\n", "");
		thread->kthread = NULL;
		return -1;
	}
	else
	{
		thread->kthread->policy = SCHED_FIFO;
		thread->kthread->static_prio = MAX_USER_RT_PRIO-1;
		thread->kthread->prio = thread->kthread->prio;
		debug_print("create slave thread for adapter %d - buffer size: %u; timeout: %llu ns\n", i2c_adapter->nr, I2C_SLAVE_BUFFER_MAX_SIZE, slave_timeout_ns );
	}

	goto addSlave;
}

// if this is the last slave being removed from the adapter, this function releases the i2c_adapter
// relases by unlocking it, and calling "post_xfer", which disables the GPIO access functions
// the monitoring thread is stopped
// if there are more slaves still attached they remain untouched and the thread continues to handle those
void unsetupAdapter(struct i2c_adapter* i2c_adapter, struct i2c_slave_instance* remSlave)
{
	i2c_slave_thread* thread = &threads[i2c_adapter->nr];

	thread->numSlaves--;
	thread->slaves[remSlave->addr] = NULL;

	debug_print("slave 0x%02x dettached from thread '%s' now with %d slaves remaining\n", remSlave->addr, thread->kthread->comm, thread->numSlaves);

	if(thread->numSlaves == 0) // if we removed the last slave from the adapter, we must free it
	{
		debug_print("stopping thread '%s'\n", thread->kthread->comm);
		kthread_stop(thread->kthread);
		thread->kthread = NULL;

		// release resources alloc'd by de adapter's underlying bus driver for gpio access
		if (thread->algo_data)
			if (thread->algo_data->post_xfer)
				thread->algo_data->post_xfer(thread->adapter);

		// release it for public use
		i2c_unlock_bus(i2c_adapter, I2C_LOCK_SEGMENT);
		debug_print("adapter %d is unlocked from slave mode\n", i2c_adapter->nr);
	}
}

// prepares a slave i2c_slave_instance
// - allocates the memory for the structure
// - makes sure the adapter is configured properly
// - creates a thread to loop reading the I2C lines and buffering input/transmitting on request
struct i2c_slave_instance* i2c_slave_bitbang_instantiate(struct i2c_adapter* i2c_adap, struct i2c_algo_bit_data *algo_data, uint8_t my_addr)
{
	struct i2c_slave_instance* slave;

	// sanity check
	if (i2c_adap == NULL || algo_data == NULL)
		return NULL;

	// create a struct that represents a slave
	slave = (struct i2c_slave_instance*) kzalloc(sizeof(struct i2c_slave_instance), GFP_KERNEL);
	if (slave == NULL)
		return NULL;

	// configures the slave
	slave->addr = my_addr;
	slave->rxPosition = 0;
  slave->txPosition = 0;
	spin_lock_init(&slave->buffer_lock);			// this lock prevents the slave thread and the read/write system calls to use the buffer at the same time
	init_waitqueue_head(&slave->wait_queue);

	if (setupAdapter(i2c_adap, algo_data, slave) != 0) // make sure the adapter is properly configured for us and the thread is running
	{
		kfree(slave);
		return NULL;
	}

	return slave;
}

// releases all resources used by the slave (frees memory)
// terminate the thread (if removing the last slave)
// unlocks the adapter hardware (if removing the last slave)
void i2c_slave_bitbang_destroy(struct i2c_adapter* i2c_adap, struct i2c_slave_instance* slave)
{
	unsetupAdapter(i2c_adap, slave);
	debug_print("destroy slave listening at addr 0x%02x", slave->addr);
	kfree(slave);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luiz Gustavo Pfitscher e Feldmann");

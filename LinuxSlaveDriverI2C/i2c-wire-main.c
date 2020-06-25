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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include "i2c-wire.h"

#define PRINT_PRELUDE "i2c-wire: "

static void printFunctionality(struct i2c_adapter * adap)
{
  #define print_func(name) printk( KERN_NOTICE PRINT_PRELUDE "\t\t" name)

  printk( KERN_NOTICE PRINT_PRELUDE "\tFUNCTIONS:\n");

  if (i2c_check_functionality(adap, I2C_FUNC_I2C))
  { print_func("I2C_FUNC_I2C");  }

  if (i2c_check_functionality(adap, I2C_FUNC_10BIT_ADDR))
  { print_func("I2C_FUNC_10BIT_ADDR");  }

  if (i2c_check_functionality(adap, I2C_FUNC_PROTOCOL_MANGLING))
  { print_func("I2C_FUNC_PROTOCOL_MANGLING");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_PEC))
  { print_func("I2C_FUNC_SMBUS_PEC");  }

  if (i2c_check_functionality(adap, I2C_FUNC_NOSTART))
  { print_func("I2C_FUNC_NOSTART");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SLAVE))
  { print_func("I2C_FUNC_SLAVE");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_BLOCK_PROC_CALL))
  { print_func("I2C_FUNC_SMBUS_BLOCK_PROC_CALL");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_QUICK))
  { print_func("I2C_FUNC_SMBUS_QUICK");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE))
  { print_func("I2C_FUNC_SMBUS_READ_BYTE");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_WRITE_BYTE))
  { print_func("I2C_FUNC_SMBUS_WRITE_BYTE");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BYTE_DATA))
  { print_func("I2C_FUNC_SMBUS_READ_BYTE_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_WRITE_BYTE_DATA))
  { print_func("I2C_FUNC_SMBUS_WRITE_BYTE_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_WORD_DATA))
  { print_func("I2C_FUNC_SMBUS_READ_WORD_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_WRITE_WORD_DATA))
  { print_func("I2C_FUNC_SMBUS_WRITE_WORD_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_PROC_CALL))
  { print_func("I2C_FUNC_SMBUS_PROC_CALL");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_BLOCK_DATA))
  { print_func("I2C_FUNC_SMBUS_READ_BLOCK_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_WRITE_BLOCK_DATA))
  { print_func("I2C_FUNC_SMBUS_WRITE_BLOCK_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
  { print_func("I2C_FUNC_SMBUS_READ_I2C_BLOCK");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_WRITE_I2C_BLOCK))
  { print_func("I2C_FUNC_SMBUS_WRITE_I2C_BLOCK");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_HOST_NOTIFY))
  { print_func("I2C_FUNC_SMBUS_HOST_NOTIFY");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE))
  { print_func("I2C_FUNC_SMBUS_BYTE");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_BYTE_DATA))
  { print_func("I2C_FUNC_SMBUS_BYTE_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_WORD_DATA))
  { print_func("I2C_FUNC_SMBUS_WORD_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_BLOCK_DATA))
  { print_func("I2C_FUNC_SMBUS_BLOCK_DATA");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_I2C_BLOCK))
  { print_func("I2C_FUNC_SMBUS_I2C_BLOCK");  }

  if (i2c_check_functionality(adap, I2C_FUNC_SMBUS_EMUL))
  { print_func("I2C_FUNC_SMBUS_EMUL");  }

}

static void printAdapterInfo(struct i2c_adapter * adap)
{
  // SANITY CHECK
  if (adap == NULL)
    return;

  printk( KERN_NOTICE PRINT_PRELUDE "ADAPTER #%d: ", adap->nr);

  // PRINT ADAPTER NAME
  if (adap->name == NULL)
    return;
  else
    printk( KERN_NOTICE PRINT_PRELUDE "\tNAME: %s\n", adap->name);

  // PRINT ADAPTER DEVICE NAME
  printk( KERN_NOTICE PRINT_PRELUDE "\tDEVICE: %s\n", dev_name(&adap->dev));

  // PRINT DEVICE PARENT
  if (adap->dev.parent == NULL)
    printk( KERN_NOTICE PRINT_PRELUDE "\tNO PARENT");
  else
    printk( KERN_NOTICE PRINT_PRELUDE "\tPARENT: %s\n", dev_name(adap->dev.parent));

  // GET AND PRINT DRIVER
  if (adap->dev.driver != NULL)
  {
    if (adap->dev.driver->name != NULL)
      printk( KERN_NOTICE PRINT_PRELUDE "\tDRIVER: %s\n", adap->dev.driver->name);
    if (adap->dev.driver->owner != NULL)
      printk( KERN_NOTICE PRINT_PRELUDE "\tMODULE: %s\n", adap->dev.driver->owner->name);
  }

  if (adap->dev.parent != NULL)
  {
    if (adap->dev.parent->driver != NULL)
    {
      if (adap->dev.parent->driver->name != NULL)
        printk( KERN_NOTICE PRINT_PRELUDE "\tDRIVER (PARENT): %s\n", adap->dev.parent->driver->name );
      if (adap->dev.parent->driver->owner != NULL)
        printk( KERN_NOTICE PRINT_PRELUDE "\tMODULE (PARENT): %s\n", adap->dev.parent->driver->owner->name);
    }
  }

  // print algo info
  if (adap->algo != NULL)
  {
    char* algoname = "UNKNOWN";
    if  (adap->algo == &i2c_bit_algo)
    {
        if ( ((struct i2c_algo_bit_data*)adap->algo_data)->getscl == NULL)
          algoname = "BIT-BANG (NO SCL READ)";
        else
          algoname = "BIT-BANG (WITH SCL READ)";
    }

    printk( KERN_NOTICE PRINT_PRELUDE "\tALGO: %s", algoname);
    printk( KERN_NOTICE PRINT_PRELUDE "\t\tMASTER-XFER: %s", (adap->algo->master_xfer != NULL) ? "SET" : "NOT SET" );
  }

  // print recover info
  printk( KERN_NOTICE PRINT_PRELUDE "\tRECOVERY-INFO: %s", (adap->bus_recovery_info != NULL) ? "SET" : "NOT SET" );
}

static dev_t i2c_wire_first = -1;
static struct class *i2c_wire_class;
static wire_device* list_wires[MAX_DEVICES_COUNT];

static int setAdapterMode(struct wire_device* wire, enum wire_mode new_mode)
{
  // check the config
  switch (new_mode)
  {
    case WIRE_MODE_MASTER_DEFAULT:
      ACCEPT_MODE:
      wire->mode = new_mode;
      return 0;
      break;

    case WIRE_MODE_SLAVE_BITBANG: // fall through
    case WIRE_MODE_MASTER_BITBANG:
      if ((wire->flags & WIRE_SUPPORT_BITBANG) || (wire->flags & WIRE_SUPPORT_RECOVERY))
        goto ACCEPT_MODE;
      else
        return -ENOPROTOOPT;
      break;

    case WIRE_MODE_SLAVE_REGISTER:
      if (wire->flags & WIRE_SUPPORT_SLAVE_REGISTER)
        goto ACCEPT_MODE;
      else
        return -ENOPROTOOPT;
      break;

    default:
      return -EPROTONOSUPPORT;
      break;
  }
}

ssize_t fops_write(struct file* file_ptr, const char *user_buffer, size_t count, loff_t *position)
{
  int retval;
  char *localbuf;
  FILE_GET_WIRE(file_ptr, openwire);

  printk( KERN_NOTICE PRINT_PRELUDE "adapter %d: write %lu bytes at offset %llu\n", openwire->wiredev->adap->nr, count, *position);

  // limit the size of the operation
  if (count > READ_WRITE_MAX_COUNT)
		count = READ_WRITE_MAX_COUNT;

  // copy the user buffer to kernel memory
  localbuf = memdup_user(user_buffer, count);
  if (IS_ERR(localbuf))
    return PTR_ERR(localbuf);

  if (openwire->addr)
  {
    // write to a specfic address
    retval = i2c_wire_operation(openwire, localbuf, count, 0);
    if (retval == 0)
      retval = -EIO; // if we let it be 0, then "echo" will repeat and it will loop forever
  }
  else
  {
    // write configuration to file
    if (*position > 0)
      retval = 0; // can only write at beginning
    else
    {
      // read the config from user input
      enum wire_mode new_mode;
      if (sscanf(localbuf, "%d", (int*)&new_mode) != 1)
        retval = -EINVAL; // argument is not valid
      else
      {
        if ((retval = setAdapterMode(openwire->wiredev, new_mode)) == 0)
          retval = count; // if there was no error setting the mode, we return number of bytes; otherwise retval holds negative error code
      }
    }
  }

  // move the buffer pointer the number of bytes written
  if (retval > 0) // not error
    *position += retval;

  kfree(localbuf);
  return retval;
}

ssize_t fops_read (struct file *file_ptr, char __user *user_buffer, size_t count, loff_t *position)
{
  int retval = 0;
  char *localbuf;
  FILE_GET_WIRE(file_ptr, openwire);

  printk( KERN_NOTICE PRINT_PRELUDE "adapter %d: read %lu bytes from offset %llu\n", openwire->wiredev->adap->nr, count, *position);

  if (count > READ_WRITE_MAX_COUNT)
    count = READ_WRITE_MAX_COUNT;

  // create a kernel memory buffer
  localbuf = kzalloc(count, GFP_KERNEL);
    if (localbuf == NULL)
  		return -ENOMEM;

  if (openwire->addr)
  {
    // send message to one specfic address
    retval = i2c_wire_operation(openwire, localbuf, count, 1);
    if (retval >= 0) // if not an error
      count = retval; // then we returned the number of bytes read
  }
  else
  {
    // we are acessing the configuration
    if (*position == 0)
      count = sprintf(localbuf, "%d", openwire->wiredev->mode);
    else
      count = 0;
  }

  // now we must copy the received data from the local kernel memory buffer to userland buffer
  if (retval >= 0) // if not an error ...
  {
    // ... copy data to user buffer
    if ((retval = copy_to_user(user_buffer, localbuf, count)) == 0)
    {
     retval = count; // all bytes where read successfully
     *position += count;
    }
    else
    {
      *position += count - retval;
      retval = -EFAULT;
    }
  }

  kfree(localbuf);
  return retval;
}

static int fops_open(struct inode *inode, struct file *file)
{
  char* filename;
  int openaddr;
  char filename_buffer[256];
  struct wire_device* wire = container_of(inode->i_cdev, struct wire_device, char_dev);

  if (wire == NULL || inode == NULL)
    return -EFAULT;

  // the name of the file being opened
  filename = dentry_path_raw(file->f_path.dentry, filename_buffer, sizeof(filename_buffer));

  // the I2C address of the file being opened - 0 for the config file
  openaddr = MINOR(inode->i_rdev) - MINOR(wire->config_device->devt);

  if (wire->opened_addr[openaddr])
  {
    printk( KERN_ERR PRINT_PRELUDE "attempt open busy file %s\n", filename);
    return -EBUSY;
  }
  else
  {
      int retval;
      wire_open_file* wop = (wire_open_file*) kzalloc(sizeof(wire_open_file), GFP_KERNEL);
      if (wop == NULL)
        return -EFAULT;

      wop->addr = openaddr;
      wop->wiredev = wire;                // what device/adapter this address is mapped to

      file->private_data = wop;           // keep track of this open file across operations
      wire->opened_addr[wop->addr] = 1;   // tell the device this specific address is open and busy

      if (wop->addr)
      {
        // the addresses different from 0 are managed by mode-specific implementation of i2c ACCESS
        // those must menage their own resources ...
        // address 0 is the config file and is managed in this "main" section of the module

        printk( KERN_NOTICE PRINT_PRELUDE "open %s (adapter %d: addr 0x%02x)\n", filename, wire->adap->nr, openaddr);
        if ((retval = i2c_wire_open(wop)) < 0)
        {
          // some error ocurred - undo the opening
          file->private_data = NULL;
          wire->opened_addr[wop->addr] = 0;
          kfree(wop);

          printk( KERN_ERR PRINT_PRELUDE "unable to open %s because mode specific opening routine returned error code %d \n", filename, retval);

          return retval;
        }
      }
      else
        printk( KERN_NOTICE PRINT_PRELUDE "open %s (adapter %d: config file)\n", filename, wire->adap->nr);
  }

  // all conditions check ... proceed
  return 0;
}

static int fops_release(struct inode *inode, struct file *file_ptr)
{
	if (file_ptr->private_data == NULL)
    return -ENXIO; // for some reason it wasn't actually open ...
  else
  {
    char filename_buffer[256];
    FILE_GET_WIRE(file_ptr, openwire);

    // close the file
    i2c_wire_release(openwire);                         // release all implementation specific resources
    openwire->wiredev->opened_addr[openwire->addr] = 0; // tell the device this file is no longer open
    file_ptr->private_data = NULL;
    kfree(openwire);

    printk( KERN_NOTICE PRINT_PRELUDE "released %s\n", dentry_path_raw(file_ptr->f_path.dentry, filename_buffer, sizeof(filename_buffer)) );

    return 0;
  }
}

static long fops_ioctl(struct file *file_ptr, unsigned int ioctl_num, unsigned long ioctl_param)
{
  FILE_GET_WIRE(file_ptr, openwire);

  if (openwire->addr)
  { // ioctl on an actual address
    // this operation is specfic to each "mode" implementation e.g. slave mode has different ioctl than master mode
    return i2c_wire_ioctl(openwire, ioctl_num, ioctl_param);
  }
  else
  { // ioctl on the configuration file
    switch (ioctl_num)
    {
      case WIRE_IOCTL_CONFIG_GET_FLAGS:
        return openwire->wiredev->flags;
      break;

      case WIRE_IOCTL_CONFIG_SET_MODE:
        return setAdapterMode(openwire->wiredev, (enum wire_mode)ioctl_param);
      break;

      default:
        return -EINVAL;
    }
  }
}

struct file_operations fops = {
	.owner = THIS_MODULE,
  .open = fops_open,
  .release = fops_release,
  .read = fops_read,
  .write = fops_write,
  .unlocked_ioctl = fops_ioctl,
};

static int createCharDeviceForAdapter(struct i2c_adapter * adap, struct device *dev_parent, enum wire_flags flags)
{
  int retval;
  dev_t config_dev_t;
  uint8_t addr;
  struct wire_device* new_wire;

  // alloc the wire
  if ((new_wire = devm_kzalloc(dev_parent, sizeof(struct wire_device), GFP_KERNEL)) == NULL)
  {
		printk( KERN_ERR PRINT_PRELUDE "devm_kzalloc failed\n");
		return -ENOMEM;
	}

  // init a chardev for the wire
  config_dev_t = MKDEV(MAJOR(i2c_wire_first), I2C_NUM_ADDRESSES*adap->nr);
	cdev_init(&new_wire->char_dev, &fops);

  if ( (retval = cdev_add(&new_wire->char_dev, config_dev_t, I2C_NUM_ADDRESSES)) != 0)
  {
    printk( KERN_ERR PRINT_PRELUDE "cdev_add failed with code %d\n", retval);

    FAULT:
    devm_kfree(dev_parent, new_wire);
    return -EFAULT;
  }

  if ((new_wire->config_device = device_create(i2c_wire_class, dev_parent, config_dev_t, NULL, "wire!%d!config", adap->nr)) == NULL)
  {
    printk( KERN_ERR PRINT_PRELUDE "device_create failed\n");
    cdev_del(&new_wire->char_dev);
    goto FAULT;
  }
  else
  {
    printk( KERN_NOTICE PRINT_PRELUDE "device \"%s\" created (MAJOR=%d minor=%d)\n", dev_name(new_wire->config_device), MAJOR(new_wire->config_device->devt), MINOR(new_wire->config_device->devt));
  }

  for (addr = 0; addr < I2C_NUM_ADDRESSES; addr++)
  {
    if (addr < 4)
      new_wire->sub_device[addr] = NULL;
    else
    {
      if ((new_wire->sub_device[addr] = device_create(i2c_wire_class, new_wire->config_device, MKDEV(MAJOR(config_dev_t), MINOR(config_dev_t) + addr), NULL, "wire!%d!0x%02x", adap->nr, addr)) == NULL)
        printk( KERN_ERR PRINT_PRELUDE "failed to create sub-device address 0x%02x\n", addr);
      else
      {
        //printk( KERN_NOTICE PRINT_PRELUDE "\tsub-device \"%s\" created (minor=%d)\n", dev_name(new_wire->sub_device[addr]), MINOR(new_wire->sub_device[addr]->devt));
      }
    }
  }
  // add other info to the wire
  new_wire->adap = adap;
  new_wire->flags = flags;
  new_wire->mode = WIRE_MODE_MASTER_DEFAULT;
  //new_wire->mode = WIRE_MODE_MASTER_BITBANG;
  //new_wire->mode = WIRE_MODE_SLAVE_BITBANG;

  list_wires[adap->nr] = new_wire;

  return 0;
}

static int removeChardev(uint8_t index)
{
  uint8_t addr;

  // sanity check
  if (index >= MAX_DEVICES_COUNT)
    return -EBADF;

  if (list_wires[index] == NULL)
    return -ENODEV;

  if (i2c_wire_class == NULL)
    return -EFAULT;

  // remove subitems
  for (addr = 0; addr < I2C_NUM_ADDRESSES; addr++)
  {
    if (list_wires[index]->sub_device[addr] != NULL)
    {
      printk( KERN_NOTICE PRINT_PRELUDE "\tremove sub-device \"%s\"\n", dev_name(list_wires[index]->sub_device[addr]));
      device_destroy(i2c_wire_class, list_wires[index]->sub_device[addr]->devt);
    }
  }

  // remove the master
  cdev_del(&list_wires[index]->char_dev);

  if (list_wires[index]->config_device != NULL)
  {
    printk( KERN_NOTICE PRINT_PRELUDE "remove device \"%s\"\n", dev_name(list_wires[index]->config_device));
    device_destroy(i2c_wire_class, list_wires[index]->config_device->devt);
  }
  else
    return -ENODEV;

  // free memory
  devm_kfree(list_wires[index]->config_device->parent, list_wires[index]);

  return 0;
}

static void initList(void)
{
  uint8_t index;
  for (index = 0; index < MAX_DEVICES_COUNT; index++)
    list_wires[index] = NULL;
}

static void purgeList(void)
{
  uint8_t index;
  for (index = 0; index < MAX_DEVICES_COUNT; index++)
    removeChardev(index);
}

static int process_i2c_adapter(struct device *dev, void *dummy)
{
	struct i2c_adapter *adap;
  enum wire_flags flags;

  if (dev == NULL)
    return 0;

	if (dev->type != &i2c_adapter_type)
		return 0;

	adap = to_i2c_adapter(dev);
  if (adap == NULL)
    return 0;

  // print adapter number and basic information
  printAdapterInfo(adap);

  // determine what it supports
  if (adap->algo != NULL)
  {
    if (adap->algo->master_xfer != NULL && i2c_check_functionality(adap, I2C_FUNC_I2C) && i2c_check_functionality(adap, I2C_FUNC_PROTOCOL_MANGLING) && i2c_check_functionality(adap, I2C_FUNC_NOSTART))
      flags |= WIRE_SUPPORT_MANGLING;

    if (adap->algo->reg_slave != NULL && i2c_check_functionality(adap, I2C_FUNC_SLAVE))
      flags |= WIRE_SUPPORT_SLAVE_REGISTER;

    if (adap->algo == &i2c_bit_algo)
      flags |= WIRE_SUPPORT_BITBANG;
  }

  if (adap->bus_recovery_info != NULL)
    if (adap->bus_recovery_info->get_scl != NULL && adap->bus_recovery_info->set_scl != NULL && adap->bus_recovery_info->get_sda != NULL && adap->bus_recovery_info->set_sda != NULL)
      flags |= WIRE_SUPPORT_RECOVERY;

  // print support info
  printk( KERN_NOTICE PRINT_PRELUDE "\tSUPPORTS: %s%s%s%s",
  (flags & WIRE_SUPPORT_MANGLING) ? "MANGLING " : "",
  (flags & WIRE_SUPPORT_SLAVE_REGISTER) ? "SLAVE " : "",
  (flags & WIRE_SUPPORT_RECOVERY) ? "BUS RECOVERY " : "",
  (flags & WIRE_SUPPORT_BITBANG) ? "BITBANGING " : "");

  createCharDeviceForAdapter(adap, dev, flags);

  // print functionality
  printFunctionality(adap);

  return 0;
}

static int __init my_i2c_module_init(void)
{
  int returncode;

  // tell the user we did init
  printk( KERN_NOTICE PRINT_PRELUDE "init ...\n");

  // clear the list of wires in this module
  initList();

  // allocate a region of char devices to match the adapters
	returncode = alloc_chrdev_region(&i2c_wire_first, 0, CHARDEV_REGION_COUNT, "i2c-wire");
	if (returncode)
  {
    printk( KERN_ERR PRINT_PRELUDE "alloc_chrdev_region failed with error code %d \n", returncode);
    goto out;
  }
  else printk( KERN_NOTICE PRINT_PRELUDE "registered range: MAJOR(%d) count = %d\n", MAJOR(i2c_wire_first), CHARDEV_REGION_COUNT);

	i2c_wire_class = class_create(THIS_MODULE, "i2c-wire");
	if (IS_ERR(i2c_wire_class))
  {
		returncode = PTR_ERR(i2c_wire_class);
    printk( KERN_ERR PRINT_PRELUDE "class_create failed with error code %d \n", returncode );
		goto out_unreg_chrdev;
	}
  else printk( KERN_NOTICE PRINT_PRELUDE  "created class %s", i2c_wire_class->name);

  // for each i2c device, find the adapters and process them
  i2c_for_each_dev(NULL, process_i2c_adapter);

  printk( KERN_NOTICE PRINT_PRELUDE "... init complete!\n");
	return 0;

out_unreg_chrdev:
  unregister_chrdev_region(i2c_wire_first, CHARDEV_REGION_COUNT);
out:
	return returncode;
}

static void __exit my_i2c_module_exit(void)
{
  printk( KERN_NOTICE PRINT_PRELUDE "exiting ...\n");

  // remove all char_devices in the list and free the memory
  purgeList();

  class_destroy(i2c_wire_class);
	unregister_chrdev_region(i2c_wire_first, CHARDEV_REGION_COUNT);

  printk( KERN_NOTICE PRINT_PRELUDE "... exited!\n");
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Luiz Gustavo Pfitscher e Feldmann");
module_init(my_i2c_module_init);
module_exit(my_i2c_module_exit);

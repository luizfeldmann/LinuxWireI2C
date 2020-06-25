# =============================================================================
# Copyright (C) 2020 Luiz Gustavo Pfitscher e Feldmann
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
# =============================================================================

TARGET_MODULE:=i2c-wire
INSTALL_PATH:=/lib/modules/$(shell uname -r)/kernel/drivers/i2c

ifneq ($(KERNELRELEASE),)
	obj-m := i2c-wire.o
	i2c-wire-objs := i2c-wire-main.o i2c-bit-banging.o i2c-wire-operations.o i2c-slave-default.o

else
	BUILDSYSTEM_DIR?=/lib/modules/$(shell uname -r)/build
	PWD:=$(shell pwd)

all :
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(BUILDSYSTEM_DIR) M=$(PWD) clean

load:
	insmod ./$(TARGET_MODULE).ko

unload:
	rmmod ./$(TARGET_MODULE).ko

install:
	cp ./$(TARGET_MODULE).ko $(INSTALL_PATH)
	depmod
	modprobe $(TARGET_MODULE)
	echo $(TARGET_MODULE) >> /etc/modules

uninstall:
	modprobe -r $(TARGET_MODULE)
	rm $(INSTALL_PATH)/$(TARGET_MODULE).ko
	depmod
	sed -i '/$(TARGET_MODULE)/d' /etc/modules

endif

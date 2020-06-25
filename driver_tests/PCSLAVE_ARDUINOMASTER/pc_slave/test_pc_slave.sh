#! /bin/sh

gcc pc_slave.c -o pc_slave || exit 0
sudo chmod a+x pc_slave || exit 0
sudo ./pc_slave
rm pc_slave

#!/bin/sh

make || exit 1
make unload
make load || exit 1
lsmod | grep i2c
watch -n 0.5 "dmesg | tail -n 20"

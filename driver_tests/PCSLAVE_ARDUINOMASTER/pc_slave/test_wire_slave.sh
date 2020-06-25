#! /bin/sh

g++ -c wire_slave.cpp -o wire_slave.o || exit 0
g++ -c ../../../LinuxWire/LinuxWire.cpp -o LinuxWire.o || exit 0
g++ wire_slave.o LinuxWire.o -o wire_slave || exit 0
rm wire_slave.o
rm LinuxWire.o

sudo chmod a+x wire_slave || exit 0
sudo ./wire_slave
rm wire_slave

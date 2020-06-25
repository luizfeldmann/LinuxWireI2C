#! /bin/sh

g++ -c wire_master.cpp -o wire_master.o || exit 0
g++ -c ../../../LinuxWire/LinuxWire.cpp -o LinuxWire.o || exit 0
g++ wire_master.o LinuxWire.o -o wire_master || exit 0
rm wire_master.o
rm LinuxWire.o


sudo chmod a+x wire_master || exit 0
sudo ./wire_master
rm wire_master

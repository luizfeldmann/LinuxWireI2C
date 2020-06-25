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

/// WARNING: please add the line "#define WIRE_DEFAULT_ADAPTER <number of your i2c adapter>" before your include statement


#ifndef Linux_Wire_h
#define Linux_Wire_h

#include <stdint.h>
#include <stddef.h>
#include "../LinuxSlaveDriverI2C/i2c-wire-config.h"

#define BUFFER_LENGTH 256

class LinuxWire
{
private:
    enum lin_wire_mode {MODE_SLAVE, MODE_MASTER};
    enum lin_wire_mode mode;

    static void (*cb_onReceive)(int);
    static void (*cb_onRequest)(void);

    uint8_t addr;
    uint8_t adapterNr;
    int i2c_file;

    int masterAvailable;
    uint8_t masterBuffer[BUFFER_LENGTH];

    static void sighandler_onReceive(int);
    static void sighandler_onRequest(int);

    int configAdapterMode(enum wire_mode newmode, enum wire_mode fallbackmode);
    int registerSignalHandlers(void);
    int openI2CaddrDev(uint8_t addr);

public:
    // CONSTRUCTOR AND DESTRUCTOR
    LinuxWire(uint8_t adap);
    ~LinuxWire();

    // INITIALIZATION
    int begin();
    int begin(uint8_t);
    int begin(int);

    int beginTransmission(uint8_t);
    int beginTransmission(int);

    int endTransmission(void);

    int requestFrom(uint8_t address, size_t size);
    int requestFrom(uint8_t, uint8_t);
    int requestFrom(int, int);

    // STREAM FUNCTIONS
    virtual int write(uint8_t);
    virtual int write(const uint8_t *, size_t);
    virtual int write(const char *format, ...);

    virtual int available(void);
    virtual int read(void);
    virtual int read(uint8_t *data, size_t quantity);
    virtual int flush(void);

    // REGISTER CALLBACKS
    int onReceive(void (*)(int));
    int onRequest(void (*)(void));
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_LINUXWIRE) && !defined(NO_GLOBAL_TWOWIRE)
extern LinuxWire Wire;
#endif

#endif

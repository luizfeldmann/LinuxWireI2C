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

/// ************************************************* ///
/// you must include this header in userland apps
/// if you wish to change the operation mode (SLAVE/MASTER)
/// or if you wish to use IOCTL to poll or flush the buffes
/// or to register your PID to receive event signals
/// ************************************************* ///

#ifndef _I2C_WIRE_CONFIG_H_
#define _I2C_WIRE_CONFIG_H_

#define MAX_DEVICES_COUNT 20

#define I2C_WIRE_SIGNAL_RECEIVED  SIGUSR1
#define I2C_WIRE_SIGNAL_REQUESTED SIGUSR2

#define SLAVE_REQUEST_SLEEP_TIME_JIFFIES  20 // when the slave is read by the master,
                                             // a signal will be sent to the user to tell him to fill the buffer,
                                             // while the slave will clockstretch for this ammount of time

enum wire_mode {
  WIRE_MODE_MASTER_DEFAULT  = 0,
  WIRE_MODE_MASTER_BITBANG  = 1,
  WIRE_MODE_SLAVE_REGISTER  = 2,
  WIRE_MODE_SLAVE_BITBANG   = 3,
};

enum wire_flags {
  WIRE_SUPPORT_MANGLING         = 1 << 0,
  WIRE_SUPPORT_SLAVE_REGISTER   = 1 << 1,
  WIRE_SUPPORT_BITBANG          = 1 << 2,
  WIRE_SUPPORT_RECOVERY         = 1 << 3,
};

enum wire_ioctl_cmd {
  WIRE_IOCTL_SLAVE_POLL_RX     = 10,
  WIRE_IOCTL_SLAVE_PURGE_RX    = 15,
  WIRE_IOCTL_SLAVE_POLL_TX     = 20,
  WIRE_IOCTL_SLAVE_PURGE_TX    = 25,
  WIRE_IOCTL_SLAVE_SIGNAL_PID  = 30,
  WIRE_IOCTL_SLAVE_WAKE_UP     = 31,
  WIRE_IOCTL_CONFIG_GET_FLAGS  = 35,
  WIRE_IOCTL_CONFIG_SET_MODE   = 40,
};

#endif

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

#include <stdio.h>
#include <string.h>

#define WIRE_DEFAULT_ADAPTER 4
#include "../../../LinuxWire/LinuxWire.h"

#define I2C_ADDR 0x41

void my_onReceived(int howMany)
{
  printf("\nReceived data! %d bytes available: ", howMany);
  while (Wire.available())
    printf("%c", Wire.read());
  printf("\n");
}

static int i;
const char test_data[] = "abcdefghijklmnopqrstuvwxyz0123456789";
void my_onRequested()
{
  Wire.write(test_data[i++ % strlen(test_data)]);
  //printf("\nData requested by master\n");
}

int main(int argc, char** argv)
{
  int ret;

  printf("\nListen/Write to I2C messages to slave 0x%02x\n", I2C_ADDR);

  if ((ret = Wire.begin(I2C_ADDR)) != 0)
  {
    fprintf(stderr, "\nerror %d\n", ret);
    return ret;
  }

  //Wire.write("This data is buffered to be read by master later");

  Wire.onReceive(my_onReceived);
  Wire.onRequest(my_onRequested);

  getchar(); // wait for user input to close the app

  return 0;
}

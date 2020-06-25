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

#define I2C_ADDR 0x07

int main(int argc, char** argv)
{
  int ret;

  printf("\nListen/Write to I2C messages to slave 0x%02x\n", I2C_ADDR);

  if ((ret = Wire.begin()) != 0)
  {
    fprintf(stderr, "\nerror %d\n", ret);
    return -1;
  }

  printf("\nInstructions:");
  printf("\n*Enter a line of text to send it to the slave");
  printf("\n*Enter a number to read that many bytes from slave");
  printf("\n*Enter 0 to exit");

  char buffer[64];
  while (1)
  {
    printf("\n\n>> ");
    if (fgets(buffer, sizeof(buffer), stdin))
    {
      int numBytes;

      if (sscanf(buffer, "%d", &numBytes))
      {
        if (numBytes <= 0)
          break;

        printf("\nRequesting %d bytes: ", numBytes);

        Wire.requestFrom(I2C_ADDR, numBytes);
        while (Wire.available())
          printf("%c", Wire.read());

        printf("\n");
      }
      else
      {
        printf("\nWriting %d bytes", strlen(buffer));

        Wire.beginTransmission(I2C_ADDR);
        Wire.write(buffer, strlen(buffer));
        Wire.endTransmission();
      }
    }
  }

  return 0;
}

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

#include <Wire.h>

#define THIS_ARDUINO_ADDR 0x07

void setup()
{
  Serial.begin(9600);
  Wire.begin(THIS_ARDUINO_ADDR); // start this as a slave in address THIS_ARDUINO_ADDR
  
  Wire.onReceive(receiveCallback); // function gets called when we receive a write
  Wire.onRequest(requestCallback); // function gets called when we receive a read
}


void receiveCallback(int howMany)
{
  Serial.print("\nRecv (");
  Serial.print(howMany);
  Serial.print(")PC -> Arduino: ");
  
  while (Wire.available())
  {
    Serial.print((char)Wire.read());
    Serial.print(" "); // this prevents {1, 2, 3} from being interpreted as {123} -- very dangerous
  }
  
  // log the i2c message to serial
}

// we send one character from the string every time we're read from
static int responseIndex = 0;
const static char response[] = "LUIZ GUSTAVO FELDMANN MADE THIS";
void requestCallback()
{
  char c = response[ (responseIndex++) % strlen(response) ];

  Serial.print(c);
  Wire.write(c);
}

void loop()
{
    // this program runs by callbacks
}

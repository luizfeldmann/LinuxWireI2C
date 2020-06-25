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

#define OTHER_END_ADDR 0x41 // this is the address the pc/slave will listen to

void setup()
{
  Serial.begin(9600);
  Wire.begin(); // no address means this is the master
  
  Serial.println("Instructions:");
  Serial.println("* Enter a line of text to send it to the slave");              // master writer / slave reader mode = arduino writer / pc reader
  Serial.println("* Enter a number to request that many bytes from the slave");  // master reader / slave writer mode = arduino reader / pc writer
}

// WARNING: this code has no overflow protection

uint8_t bufferpos = 0;
char buffer[256];
void loop()
{
  // wait until there is serial data to be read
  if (!Serial.available())
    return;

  char ch = Serial.read(); // read one character
  if (ch == '\r') // ignore carret returns
    return;
    
  if (ch != '\n') // buffer all characters received, except for the <enter>, which will be used to commit the message
  {
    buffer[bufferpos] = ch;
    buffer[bufferpos + 1] = '\0';  // we add a terminator('\0') to make sure the buffer always contains a properly formatted string
    bufferpos++;
  }
  else
  {
    int numBytes = 0;
    bufferpos = 0; // reset the buffer
    
    if (sscanf(buffer, "%d", &numBytes))
    {
      // user asked for a number of bytes ...
      if (numBytes < 0 || numBytes > 32)
      {
        Serial.print("\nEnter a a positive number below 32; you entered ");
        Serial.print(numBytes);
        return;
      }
      
      // performs a "master read" operation and returns the number of bytes read
      int recvlen = Wire.requestFrom(OTHER_END_ADDR, numBytes, 1);
      
      // informs the user what happended
      Serial.print("\nRequested ");
      Serial.print(numBytes);
      Serial.print(" from slave; received ");
      Serial.print(recvlen);
      Serial.print(" bytes\n>> ");
      
      // prints the message
      while (Wire.available() && recvlen--)
        Serial.print((char)Wire.read());
    }
    else
    {
      // user entered a line of text to be sent to the slave....
      
      // send an I2C message
      Wire.beginTransmission(OTHER_END_ADDR);        // starts buffering the data
      Wire.write((uint8_t*)buffer, strlen(buffer)); // adds data to buffer, does not actually send
      Wire.endTransmission();                      // transmits the buffer via I2C
      
      // and print the message to the serial as a "log"
      Serial.print("\nArduino -> PC: ");
      Serial.print(buffer);
    }
  }
}

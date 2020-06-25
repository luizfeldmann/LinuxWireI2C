# LINUX WIRE

This library aims to implement a Linux port of the Wire library used in Arduino.

It works in conjunction with a custom kernel module you'll find in the root folder of this repo.

#### Master mode
In this mode you initiate the transaction by reading or writing from a slave at any time.

To choose master mode you begin the Wire without address (or with 0):
```c
Wire.begin();
```

You can send data to a slave at any time:
```c
Wire.beginTransmission(0x50);
Wire.write(buffer, strlen(buffer));
Wire.endTransmission();
```

You can also request to read some data:
```c
Wire.requestFrom(0x50, howMany);
```

The data will be buffered and can be processed later:
```c
while (Wire.available())
  printf("%c", Wire.read());
```

#### Slave mode
This mode cannot initiate a transaction. You can receive data only when the master writes, or reply only when the master requests.

Initialize the Wire with the desired address and set the callbacks to handle the events:
```c
Wire.begin(0x27);


Wire.onReceive(my_onReceived);
Wire.onRequest(my_onRequested);
```
This function will be called **after** a whole message is received:
```c
void my_onReceived(int howMany)
{
  printf("\nReceived data! %d bytes available: ", howMany); // howMany contains the value of Wire.available()

  while (Wire.available())
    printf("%c", Wire.read()); // print all the received bytes

  printf("\n");
}
```
This function will be called **before** data is sent, so you can fill the response inside it:
```c
void my_onRequested()
{
  Wire.write("Hello world from Linux!");
}
```

#### Warning:
If you wish to use the global extern variable **Wire** you must define the number of the I2C adapter beforeyou include the library:
```c
#define WIRE_DEFAULT_ADAPTER 4
#include "LinuxWire.h"

...

Wire.begin();
```
Alternatively, you can declare another instance and choose the adapter dynamically:
```c
LinuxWire myWire = new LinuxWire(adapterNumber);

myWire.begin();
```

#### CHANGELOG
**Jun 2020** - Project created

#### Known bugs
...

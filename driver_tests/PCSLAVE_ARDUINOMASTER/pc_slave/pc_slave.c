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
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <sys/ioctl.h>

#define I2C_ADAP 4
#define I2C_ADDR 0x41

unsigned char configure(char* path, int mode)
{
  int fp;

  if ((fp = open(path, O_RDWR)) < 0)
  {
    fprintf(stderr, "\nconfiguration: error opening %s: %d %s\n", path, errno, strerror(errno));
    return 0;
  }

  char param[3];
  sprintf(param, "%d", mode);

  if (write(fp, param, strlen(param)) <= 0)
  {
    fprintf(stderr, "\nconfiguration: error writing value %d to %s: %d %s\n", mode, path, errno, strerror(errno));
    close(fp);
    return 0;
  }

  close(fp);
  return 1;
}

int i2c;

void sig_onReceived(int signo)
{
    printf("\nEvent: received %d bytes!\n", ioctl(i2c, 10, 0));
}

void sig_onRequested(int signo)
{
  printf("\nEvent: data requested!\n");
}

int main(int argc, char** argv)
{
    char device_path[256];
    char config_path[256];

    sprintf(device_path, "/dev/wire/%d/0x%02x", I2C_ADAP, I2C_ADDR);
    sprintf(config_path, "/dev/wire/%d/config", I2C_ADAP);

    if (!configure(config_path, 3))
      return -1;

    if ((i2c = open(device_path, O_RDWR)) < 0)
    {
      fprintf(stderr, "\nerror opening %s: %d %s\n", device_path, errno, strerror(errno));
      return -1;
    }

    if (signal(SIGUSR1, sig_onReceived) == SIG_ERR)
        fprintf(stderr, "\ncan't catch SIGUSR1\n");

    if (signal(SIGUSR2, sig_onRequested) == SIG_ERR)
        fprintf(stderr, "\ncan't catch SIGUSR2\n");

    if (ioctl(i2c, 30, getpid()))
        fprintf(stderr, "\nfailed to register pid to slave\n");

    printf("\nListen/Write to I2C messages to slave %s; configured %s\n", device_path, config_path);

    void writeMode()
    {
      char buff[128];

      while (1)
      {
        printf("\nWRITE >> ");
        fflush(stdin);
        if (fgets(buff, sizeof(buff), stdin))
        {
          if (!strlen(buff))
            break;

          int i = write(i2c, buff, strlen(buff));

          if (i > 0)
            printf("BUFFERED (%d bytes) >> %.*s", i, i, buff);
        }
      }
    }

    void readMode()
    {
      int recvlen;
      char buffer[256];
      while (1)
      {
        int readlen;

        printf("\nRead what length (0 to exit)? ");

        if (!scanf("%d", &readlen))
          break;

        if (readlen == 0)
          break;

        printf("\nAttemp read %d bytes from %s:", readlen, device_path);

        if ((recvlen = read(i2c, buffer, readlen)) < 0)
        {
          printf("\n%d %s",errno, strerror(errno));
          continue;
        }
        else if (recvlen == 0)
          printf("\n>> no data\n");
        else
          printf("\n>> %.*s\n", recvlen, buffer);
      }
    }

    while (1)
    {
      char mode;
      printf("\n\nMode (R / W) ?");
      scanf(" %c", &mode);

      if (mode == 'w' || mode == 'W')
        writeMode();
      else if (mode == 'r' || mode == 'R')
        readMode();
      else if (mode == 'q' || mode == 'Q')
        break;
    }

    close(i2c);

    return 0;
}

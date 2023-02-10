/**
 * @file uart_communication.cpp
 * @author duckstarr
 * @brief Class used to configure UART protocol and enable read/write to the bus
 */

#include <uart.hpp>

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <sys/ioctl.h>

namespace sensor
{
  UART::UART(const char* iDevicePath)
    : mDevicePath(iDevicePath)
  {
    mFd = SetupInterface();
  }

  UART::~UART()
  {
    close(mFd);
  }

  int UART::SetupInterface()
  {
    int wFd;

    if ((wFd = open(mDevicePath, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) 
    {
      printf("UART::SetupInterface - Unable to open UART device [%s].\n", mDevicePath);
      exit(-1);
    }
    else 
    {
      fcntl(wFd, F_SETFL, 0);
    }

    struct termios tty;

    tcgetattr(wFd, &tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // Canonical mode
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;         /* 8-bit characters */
    tty.c_cflag &= ~PARENB;     /* no parity bit */
    tty.c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
    tty.c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

    tty.c_lflag |= ICANON | ISIG;  /* canonical input */
    tty.c_lflag &= ~(ECHO | ECHOE | ECHONL | IEXTEN);
    
    tty.c_iflag &= ~INPCK;
    tty.c_iflag |= ICRNL;
    tty.c_iflag &= ~(INLCR | IGNCR | IUCLC | IMAXBEL);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);   /* no SW flowcontrol */

    tty.c_oflag &= ~OPOST;

    tcsetattr (wFd, TCSANOW, &tty);

    return wFd;
  }

  ssize_t UART::command(const void* iBuffer, size_t iNBytes)
  {
    return write(mFd, iBuffer, iNBytes);
  }

  ssize_t UART::receive(void* iBuffer, size_t iNBytes)
  {
    return read(mFd, iBuffer, iNBytes);
  }
}

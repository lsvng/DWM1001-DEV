/**
 * @file uart_communication.hpp
 * @author duckstarr
 * @brief Class used to configure UART protocol and enable read/write to the bus
 */

#ifndef UART_HPP
#define UART_HPP

#include <iostream>

namespace sensor
{
  class UART
  {
    public:
      /**
       * @brief Configure UART interface
       * 
       * @param iDevicePath    Device path (i.e., /dev/USB0)
       */
      UART(const char* iDevicePath);

      /**
       * @brief UART Destructor
       * 
       */
      ~UART();

    protected:
      /**
       * @brief Write to UART bus
       * 
       * @param iBuffer    Buffer containing a series of commands
       * @param iNBytes    Size of buffer (in bytes)
       * @return [ssize_t] Size of buffer written to the UART bus
       */
      ssize_t command(const void* iBuffer, size_t iNBytes);

      /**
       * @brief Read from UART bus
       * 
       * @param oBuffer    Response from UART bus
       * @param iNBytes    Size of buffer (in bytes)
       * @return [ssize_t] Size of buffer read from the UART bus
       */
      ssize_t receive(void* oBuffer, size_t iNBytes);

    private:
      /**
       * @brief Configure UART interface
       * 
       * @return [int] File discriptor
       */
      int SetupInterface();

    private:
      int         mFd;            // File discriptor
      const char* mDevicePath;    // Device path (i.e., /dev/USB0)
      uint8_t     mDeviceAddress; // Device UART address
  };
}

#endif /* UART_HPP */

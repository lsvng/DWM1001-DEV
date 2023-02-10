/**
 * @file dwm1001.hpp
 * @author duckstarr
 * @brief A C++ program used to interface with the DWM1001 module.
 * 
 */

#ifndef DWM1001_H
#define DWM1001_H

#include <uart.hpp>
#include <kalman_filter.hpp>

#include <unistd.h>  // UNIX standard function definitions
#include <string>
#include <cstring>
#include <sstream>
#include <thread>    // std::this_thread::sleep_for

#include <vector>

namespace sensor
{
  class dwm1001 : public UART
  {
    public:
      /**
       * @brief Construct a new dwm1001 object
       * 
       * @param device 
       * @param nominal_update_rate 
       * @param A 
       * @param C 
       * @param Q 
       * @param R 
       * @param P 
       */
      dwm1001(const char * device, const int nominal_update_rate, 
            const Eigen::MatrixXd& A,
            const Eigen::MatrixXd& C,
            const Eigen::MatrixXd& Q,
            const Eigen::MatrixXd& R,
            const Eigen::MatrixXd& P);

      /**
       * @brief Destroy the dwm1001 object
       * 
       */
      ~dwm1001();

      /**
       * @brief Disable copy constructor
       * 
       * @param dwm1001 
       */
      dwm1001(dwm1001& dwm1001) = delete;

      /**
       * Disable copy assignment
      */
      void operator=(const dwm1001&) = delete;

      /**
       * @brief Read raw data from dwm1001 dev board
       * 
       * @return int 
       */
      int readDWM1001();

      /**
       * @brief Split delimiter and parse raw data
       * 
       * @return std::vector<double> 
       */
      std::vector<double> decodeRawData();

    private:
      /**
       * @brief Filter dwm1001 data vis a Kalman filter
       * 
       * @param pose 
       * @return std::vector<double> 
       */
      std::vector<double> filterRawData(std::vector<std::string> * pose);

      /**
       * @brief Issue a command to the dwm1001 dev board
       * 
       * @param command both 'lep' OR 'les' commands are supported.
       * 
       * @link https://www.decawave.com/wp-content/uploads/2019/01/DWM1001-API-Guide-2.2.pdf @endlink
       */
      void writeDWM1001Tag();

    private:
      filter::KalmanFilter KF_X; // KF in the X domain.
      filter::KalmanFilter KF_Y; // KF in the Y domain.
      filter::KalmanFilter KF_Z; // KF in the z domain.

      int fd;                    // File Descriptor.
      int nur;                   // nominal update rate
      uint8_t buf[256];          // Raw buffer
  };
}

#endif /* DWM1001_H */

/**
 * @file dwm1001.hpp
 * @author duckstarr
 * @brief A C++ program used to interface with the DWM1001 module.
 * 
 */

#ifndef DWM1001_H
#define DWM1001_H

#include <kalman_filter.hpp>

#include <iostream>

#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */

#include <string>
#include <cstring>
#include <sstream>

#include <stdexcept> // std::invalid_argument
#include <thread>    // std::this_thread::sleep_for

#include <vector>

class dwm1001
{
  public:
    dwm1001(const char * device, const int nominal_update_rate, 
          const Eigen::MatrixXd& A,
          const Eigen::MatrixXd& C,
          const Eigen::MatrixXd& Q,
          const Eigen::MatrixXd& R,
          const Eigen::MatrixXd& P);
    ~dwm1001();

    dwm1001(dwm1001& dwm1001) = delete;
    void operator=(const dwm1001&) = delete;

    int readDWM1001();
    std::vector<double> decodeRawData();

  private:
    /**
     * @brief 
     * 
     * @param device 
     * @return int 
     */
    int SetupInterface(const char * device);

    /**
     * @brief 
     * 
     * @param pose 
     * @return std::vector<double> 
     */
    std::vector<double> filterRawData(std::vector<std::string> * pose);

    /**
     * @brief 
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

#endif /* DWM1001_H */

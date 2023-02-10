/**
 * @file dwm1001_example.cpp
 * @author duckstarr
 * @brief A C++ program used to interface with the DWM1001 module.
 * 
 */

#include <dwm1001.hpp>
#include <memory>

#include <iostream>
#include <signal.h>

using namespace sensor;

// Signal handler callback function
volatile sig_atomic_t done = 0;
void sig_handler(int signum) 
{
    done = 1;
}

int main(int argc, char ** argv)
{
  // Set up signal handler
  struct sigaction action;
  memset(&action, 0, sizeof(struct sigaction));
  action.sa_handler = sig_handler;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGINT, &action, NULL);

  int n = 3; // Number of states [position, velocity, acceleration].
  int m = 1; // Number of measurements.
  double dt = 1.0/30; // Timestamp.

  // Declare MAT for Kalman filter.
  Eigen::MatrixXd A(n, n); // System dynamics matrix
  Eigen::MatrixXd C(m, n); // Output matrix
  Eigen::MatrixXd Q(n, n); // Process noise covariance
  Eigen::MatrixXd R(m, m); // Measurement noise covariance
  Eigen::MatrixXd P(n, n); // Estimate error covariance

  // Measure the position only.
  A << 
    1, dt, 0, 
    0, 1, dt, 
    0, 0, 1;

  C << 1, 0, 0;

  // Reasonable covariance matrices
  Q << 
    .05, .05, .0, 
    .05, .05, .0, 
    .0, .0, .0;

  R << 1;

  P << 
    .1, .1, .1, 
    .1, 10000, 10, 
    .1, 10, 100;

  // Start DWM1001 module.
  std::string device = "/dev/ttyACM0"; 
  int nominal_update_rate = 100;
  
  auto dwm1001_obj = std::unique_ptr<dwm1001>(new dwm1001(device.c_str(), nominal_update_rate, A, C, Q, R, P));

  while(!done)
  {
    int ret = dwm1001_obj->readDWM1001();
    std::vector<double> pose = dwm1001_obj->decodeRawData();

    if(!pose.empty()) std::cout << "x: " << pose.at(0) << "\ty: " << pose.at(1) << "\tz: " << pose.at(2) << std::endl;
    else if(ret == -1) break;
  }

  return EXIT_SUCCESS;
}

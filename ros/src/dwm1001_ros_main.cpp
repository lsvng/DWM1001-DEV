/**
 * @file dwm1001_ros_main.cpp
 * @author duckstarr
 * @brief ROS Node for DWM1001.
 * 
 */

#include <dwm1001_ros.hpp>

#include <thread>
#include <memory>

using namespace sensor;

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "dwm1001_ros");
  ros::NodeHandle nh("~");

  std::string device;
  int nominal_update_rate;

  int n = 3; // Number of states [position, velocity, acceleration].
  int m = 1; // Number of measurements.
  double dt = 1.0/100; // Timestamp.

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

  nh.getParam("device", device);
  nh.getParam("nominal_update_rate", nominal_update_rate);

  auto dwm1001_obj = std::unique_ptr<dwm1001_ros>(new dwm1001_ros(nh, device.c_str(), nominal_update_rate, A, C, Q, R, P));

  while(ros::ok())
  {
    dwm1001_obj->publishPointStamped();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
}

/**
 * @file dwm1001_ros.cpp
 * @author duckstarr
 * @brief ROS Node for DWM1001.
 * 
 */

#include <dwm1001_ros.hpp>

namespace sensor
{
  dwm1001_ros::dwm1001_ros(ros::NodeHandle nh, const char * device, const int nominal_update_rate,
                        const Eigen::MatrixXd& A,
                        const Eigen::MatrixXd& C,
                        const Eigen::MatrixXd& Q,
                        const Eigen::MatrixXd& R,
                        const Eigen::MatrixXd& P) : nodehandle(nh), dwm1001(device, nominal_update_rate, A, C, Q, R, P)
  {
    point_publisher = nodehandle.advertise<geometry_msgs::PointStamped>("/Point", 10);

    ROS_INFO("Success - dwm1001_ros::dwm1001_ros");
  }

  dwm1001_ros::~dwm1001_ros()
  {
    nodehandle.shutdown();
    point_publisher.shutdown();

    ROS_INFO("End of ROS node.");
  }

  void dwm1001_ros::publishPointStamped()
  {
    pointStamped.header.frame_id = "dwm1001";
    pointStamped.header.stamp = ros::Time::now();

    int ret = dwm1001::readDWM1001();

    if(!dwm1001::decodeRawData().empty())
    {
      std::vector<double> pose = dwm1001::decodeRawData();
      
      pointStamped.point.x = pose.at(0);
      pointStamped.point.y = pose.at(1);
      pointStamped.point.z = pose.at(2);
    }

    std::cout << "x: " << pointStamped.point.x << "\ty: " << pointStamped.point.y << "\tz: " << pointStamped.point.z << std::endl;
    
    point_publisher.publish(pointStamped);
  }
}

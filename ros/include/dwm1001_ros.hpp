/**
 * @file dwm1001_ros.hpp
 * @author duckstarr
 * @brief ROS Node for DWM1001.
 * 
 */

#ifndef DWM1001_ROS_HPP
#define DWM1001_ROS_HPP

#include <dwm1001.hpp>

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>

class dwm1001_ros : public dwm1001
{
  public:
      /**
       * @brief Construct a new dwm1001 ros object
       * 
       * @param nh 
       * @param device 
       * @param nominal_update_rate 
       * @param A 
       * @param C 
       * @param Q 
       * @param R 
       * @param P 
       */
      dwm1001_ros(ros::NodeHandle nh, const char * device, const int nominal_update_rate,
              const Eigen::MatrixXd& A,
              const Eigen::MatrixXd& C,
              const Eigen::MatrixXd& Q,
              const Eigen::MatrixXd& R,
              const Eigen::MatrixXd& P);
    ~dwm1001_ros();

    /**
     * @brief Construct a new dwm1001 ros object
     * 
     * @param iDwm1001_ros 
     */
    dwm1001_ros(dwm1001_ros& iDwm1001_ros) = delete;

    /**
     * @brief 
     * 
     * @param iDwm1001_ros 
     */
    void operator=(const dwm1001_ros& iDwm1001_ros) = delete;

    /**
     * @brief 
     * 
     */
    void publishPointStamped();

  private:
    ros::NodeHandle nodehandle;
    ros::Publisher point_publisher;

    geometry_msgs::PointStamped pointStamped;

    // std::vector<double> pose;
};

#endif /* DWM1001_ROS_HPP */

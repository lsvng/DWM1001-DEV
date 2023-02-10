/**
 * @file dwm1001.cpp
 * @author duckstarr
 * @brief A C++ program used to interface with the DWM1001 module.
 * 
 */

#include <dwm1001.hpp>
#include <iostream>

namespace sensor
{
  dwm1001::dwm1001(const char* device,const int nominal_update_rate, 
                const Eigen::MatrixXd& A,
                const Eigen::MatrixXd& C,
                const Eigen::MatrixXd& Q,
                const Eigen::MatrixXd& R,
                const Eigen::MatrixXd& P) 
                : UART(device)
                , nur(nominal_update_rate)
                , KF_X(A, C, Q, R, P)
                , KF_Y(A, C, Q, R, P)
                , KF_Z(A, C, Q, R, P)
  {
    KF_X.init();
    KF_Y.init();
    KF_Z.init();
    
    this->writeDWM1001Tag();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  dwm1001::~dwm1001()
  {
    std::cout << "End of dwm1001 program." << std::endl;
    close(fd);
  }

  int dwm1001::readDWM1001()
  {
    int ret = receive(buf, sizeof(buf));
    std::this_thread::sleep_for(std::chrono::milliseconds(nur));
    
    if(ret == 23) buf[ret] = '\0'; // rid of garbage values.
    else if(ret == 0) this->writeDWM1001Tag();
    else memset(buf, 0, sizeof(buf));

    return ret;
  }

  void dwm1001::writeDWM1001Tag()
  {
    command("reset\r", 6);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    command("\r\r", 2); // 2
    std::this_thread::sleep_for(std::chrono::seconds(1));
    command("lep\r", 4);
  }

  std::vector<double> dwm1001::decodeRawData()
  {
    std::string token;
    std::vector<double> pose;
    std::vector<std::string> rawDataVec;
    std::istringstream ss((const char *)buf);

    // Split delimiter.
    while(std::getline(ss, token, ',')) rawDataVec.push_back(token);
    if(!rawDataVec.empty()) 
    {
        pose.assign(this->filterRawData(& rawDataVec).begin(), this->filterRawData(& rawDataVec).end());
    }

    return pose;
  }

  std::vector<double> dwm1001::filterRawData(std::vector<std::string> * pose)
  {
    std::vector<double> ret;

    if(pose->at(0) == "POS")
    {
      Eigen::VectorXd raw_x(1);
      Eigen::VectorXd raw_y(1);
      Eigen::VectorXd raw_z(1);

      raw_x << std::stod(pose->at(1));
      raw_y << std::stod(pose->at(2));
      raw_z << std::stod(pose->at(3));

      ret.push_back(KF_X.compute(raw_x, 0.1)(0));
      ret.push_back(KF_Y.compute(raw_y, 0.1)(0));
      ret.push_back(KF_Z.compute(raw_z, 0.1)(0));
      ret.push_back(std::stod(pose->at(4))); // quality
    }

    return ret;
  }
}

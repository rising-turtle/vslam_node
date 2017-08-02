/*
 * July 26 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  IMU IO, publisher
 *
 * */

#pragma once 

#include <fstream>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <ros/time.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std; 

namespace Eigen{
  typedef Matrix<double, 6, 1> Vector6d; 
}

typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > stdv_eigen_vector6d;

class CImuPub 
{
public:
  CImuPub(); 
  CImuPub(string imu_topic); 
  virtual ~CImuPub(); 
  virtual void init(string topic);          
  virtual bool readImuData(string f);       // read all imu data
  bool setStartIndex(ros::Time);            // set curIndex as start point 
  bool publishToTime(ros::Time);            // publish data until this ros::Time 
  void publishCurImu();                     // publish current imu data 
  ros::Time getLastPubTime();               // timestamp for the last published imu data
  stdv_eigen_vector6d mv_measurements;  // gx gy gz, ax ay az 
  vector<ros::Time> mv_timestamps;         // timestamps for imu measurement 
  ros::Time m_lastPubTime;                  // timestamp of the last published imu data 
  int m_curIndex;                       // point to current data
  string m_imuTopic;                    // m_imuTopic 
  ros::Publisher m_imuPub;              // imuPublisher 
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/*
 * July 26 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  IMU IO, publisher
 *
 * */

#include "imu_pub.h"
#include <sstream>
#include <sensor_msgs/Imu.h>

namespace
{
  ros::Time compT(string curr_t)
  {
    string secs = curr_t.substr(0, 10); 
    string nanosecs = curr_t.substr(11, 9); 
    ros::Time rost(std::stoi(secs), std::stoi(nanosecs)); 
    return rost; 
  }
}
CImuPub::CImuPub()
{
  init("/imu0");
}

CImuPub::CImuPub(string imuTopic)
{
  init(imuTopic); 
}

CImuPub::~CImuPub(){}

void CImuPub::init(string topic)
{
  ros::NodeHandle n; 
  int q = 70;
  m_curIndex = 0; 
  m_imuTopic = topic; 
  m_imuPub = n.advertise<sensor_msgs::Imu>(m_imuTopic, q); 
  return ; 
}

ros::Time CImuPub::getFirstStamp()
{
  if(mv_timestamps.size() <= 0)
  {
    ROS_ERROR("imu_pub.cpp: No data now, should not call getFirstStamp()"); 
    return ros::Time::now();
  }
  return mv_timestamps[0]; 
}

bool CImuPub::computeInitT(tf::Transform& T, int N)
{
  Eigen::Vector3d am; 
  if(!averageAm(N, am))
  {
    return false; 
  }
  
  double fXg, fYg, fZg; 
  double roll, pitch;  
  fXg = am(0); fYg = am(1); fZg = am(2); 
  roll  = (atan2(-fYg, fZg));
  pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg)));
 
  T.getOrigin() = tf::Vector3(0, 0, 0); 
  T.getBasis().setRPY(roll, pitch, 0); 
  return true; 
}

int CImuPub::findIndex(ros::Time t)
{
  int ret = -1; 
  for(int i=0; i<mv_timestamps.size()-1; i++)
  {
    ros::Time& ct = mv_timestamps[i]; 
    ros::Time& nt = mv_timestamps[i+1]; 
    if(ct <= t && nt > t)
    {
      ret = i;
      break; 
    }
  }
  return ret; 
}

bool CImuPub::setStartIndex(ros::Time t)
{
  m_curIndex = findIndex(t); 
  if(m_curIndex < 0)
  {
    ROS_ERROR("%s setStartIndex failed with t = %lf ", __FILE__, t.toSec());
    return false; 
  }
  return true; 
}

bool CImuPub::readImuData(string f)
{
  ifstream inf(f.c_str()); 
  
  if(!inf.is_open())
  {
    printf("%s failed to open imu file %s\n", __FILE__, f.c_str()); 
    return false; 
  }
  string t; 
  double ax, ay, az, gx, gy, gz, yaw, pitch, roll; 
  Eigen::Vector6d m; 
  Eigen::Vector3d rpy; 
  while(!inf.eof())
  {
    string line;
    getline(inf, line);
    if(line.empty())
      break; 
    stringstream ss; 
    ss << line; 
    ss>>t>>ax>>ay>>az>>gx>>gy>>gz>>yaw>>pitch>>roll; 
    
    ros::Time rt = compT(t); 
    
    m << ax, ay, az, gx , gy, gz; 
    // rpy << roll, pitch, yaw; 
    mv_measurements.push_back(m); 
    mv_timestamps.push_back(rt);
    // mv_rpy.push_back(rpy); 
  }
  printf("%s succeed to load %i imu measurements\n", __FILE__, mv_timestamps.size()); 
  inf.close(); 
  return true; 

}

bool CImuPub::publishToTime(ros::Time t)
{
  ros::Time cur_t = mv_timestamps[m_curIndex]; 
  if(cur_t > t)
  {
    ROS_WARN("%s cur_t > t, no imu available? ", __FILE__); 
    return false;
  }

  double dt = t.toSec();   
  for(;m_curIndex < mv_timestamps.size(); ++m_curIndex)
  {
    cur_t = mv_timestamps[m_curIndex]; 
    if(cur_t <= t)
    {
      // ROS_INFO_STREAM("imu_pub.cpp: publish at index: "<<m_curIndex<<" timestamp: "<<cur_t<< " <= t: "<<t<<std::fixed<<" dt = "<<dt);
      // publish
      publishCurImu(); 
    }else
    {
      publishCurImu();
      ++m_curIndex; 
      // ROS_WARN_STREAM("imu_pub.cpp: current time "<< cur_t<<" > t= "<<t<<" finished!");
      break; 
    }
  }
  return true; 
}

bool CImuPub::averageAm(ros::Time t, Eigen::Vector3d& avgm)
{
  int N = findIndex(t); 
  if(N < 0)
  {
    cerr <<" imu_pub.cpp: cannot find index at t: "<<t<<endl; 
    return false; 
  }
  return averageAm(N, avgm); 
}

bool CImuPub::averageAm(int N, Eigen::Vector3d& avgm)
{
  double ax = 0;
  double ay = 0; 
  double az = 0; 
  int i; 
  for(i=0; i<mv_timestamps.size() && i<N; i++)
  {
    Eigen::Vector6d& m = mv_measurements[i]; 
    ax += m(0); ay += m(1); az += m(2); 
  }
  if(i>0)
  {
    avgm(0) = ax/i; avgm(1) = ay/i; avgm(2) = az/i; 
  }else
  {
    cerr <<"imu_pub.cpp: what? i = "<<i<<endl; 
    return false; 
  }
  return true; 
}

void CImuPub::publishCurImu()
{
  // imu msg
  Eigen::Vector6d& m = mv_measurements[m_curIndex]; 
  sensor_msgs::Imu imu_msg; 
  imu_msg.header.stamp = mv_timestamps[m_curIndex];
  imu_msg.header.seq = m_curIndex; 
  imu_msg.linear_acceleration.x = m(0); 
  imu_msg.linear_acceleration.y = m(1);
  imu_msg.linear_acceleration.z = m(2); 

  imu_msg.angular_velocity.x = m(3); 
  imu_msg.angular_velocity.y = m(4); 
  imu_msg.angular_velocity.z = m(5); 
  
  // publish imu
  m_imuPub.publish(imu_msg); 
  m_lastPubTime = imu_msg.header.stamp; 
  ros::spinOnce(); 
  usleep(5000); // 200 hz = 5ms
  return ; 
}

ros::Time CImuPub::getLastPubTime()
{
  return m_lastPubTime;
}



/*
 *  Aug. 3 2017, He Zhang, hxzhang1@ualr.edu 
 *
 *  try to compute roll and pitch angle using acceleration 
 *
 * */

#include <iostream>
#include <tf/tf.h>
#include <cmath>
#include <ros/ros.h>
#include "imu_pub.h"

using namespace std;

#define R2D(r) (((r)*180.)/M_PI)
#define D2R(d) (((d)*M_PI)/180.)

void test();

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "test_gravity"); 
 test(); 
  return 0; 
}

void test()
{
  CImuPub imu; 
  string imu_file = "/home/davidz/work/data/ETAS_2F_640_30/imu_vn100.log"; 
  if(!imu.readImuData(imu_file))
  {
    cout << "failed to load imu_file "<<imu_file<<endl;
    return ;
  }
  
  Eigen::Vector3d am; 
  double fXg, fYg, fZg; 
  double roll, pitch;  
  tf::Vector3 g(0, 0, 9.81); 
  if(imu.averageAm(10, am))
  {
    cout <<" succeed to compute avg_am: ax "<<am(0)<<" ay "<<am(1)<<" az "<<am(2)<<endl; 
    // compute roll and pitch 
    fXg = am(0); fYg = am(1); fZg = am(2); 
    roll  = (atan2(-fYg, fZg));
    pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg)));
    cout <<"roll = "<<R2D(roll)<<" pitch: "<<R2D(pitch)<<endl; 
    tf::Transform T; 
    T.getOrigin() = tf::Vector3(0, 0, 0);
    T.getBasis().setRPY(roll, pitch, 0);
    tf::Vector3 gs1 = T*g;
    tf::Vector3 gs2 = T.inverse()*g; 
    cout <<"gs1 = "<<gs1.x()<<" "<<gs1.y()<<" "<<gs1.z()<<endl; 
    cout <<"gs2 = "<<gs2.x()<<" "<<gs2.y()<<" "<<gs2.z()<<endl; 
    tf::Vector3 ga(am(0), am(1), am(2)); 
    tf::Vector3 gg = T.inverse()*ga; 
    cout <<"gg = "<<gg.x()<<" "<<gg.y()<<" "<<gg.z()<<endl; 
  }
  
  return ; 
}



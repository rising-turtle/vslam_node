/*
 *  Nov. 7, 2016, David Z
 *
 * interface for imu_vn100 
 *
 * */

#ifndef IMU_VN100_H
#define IMU_VN100_H

// #include "imu_base.h"

#include <string>
#include <vector>
#include <Eigen/Core>
#include <cmath>

#define D2R(d) (((d)*M_PI)/180)
#define R2D(r) (((r)*180)/M_PI)

namespace Eigen{
  typedef Matrix<double, 3, 1> Vector3d; 
  typedef Matrix<double, 6, 1> Vector6d; 
}

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > stdv_eigen_vector3d;
typedef std::vector<Eigen::Vector6d, Eigen::aligned_allocator<Eigen::Vector6d> > stdv_eigen_vector6d; 

class CImuVn100
{
  public:
    CImuVn100(double dt=0.005); 
    virtual ~CImuVn100();
    // virtual void setStartPoint(double t); 
    virtual bool readImuData(std::string f);         // read all imu data
    // bool getRPYAt(double t, Eigen::Vector3d& rpy);  // 
    // boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> getIMUParams(); // parameters for VN100 
    
    // void resetGravity(int n = 1);  // For the first frame, camera may not be vertical to the ground, use the first n readings to estimate initial gravity  

    double getCurrTimeStamp(); 
    void getCurrMeasurement(Eigen::Vector6d& ); 
    bool moveNext(); 
    int mv_index; 
    stdv_eigen_vector6d mv_measurements; 
    stdv_eigen_vector3d mv_rpy; // rpy 
    std::vector<double> mv_timestamps; // timestamps
    Eigen::Vector3d mp_ini_rpy; // initial rpy 

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};



#endif

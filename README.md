# vslam_node
A collection of wrappers (ROS nodes) and tools to test visual / visual-inertial SLAM algorithms 

The code that can prepare the datasets that required by the following packages can be found in https://github.com/rising-turtle/img_pub.

## ORB_VIO
The original VIORB package is https://github.com/jingpang/LearnVIORB
To enable the system starts from the first frame, use depth to initialize features just for the first frame, 
modification are made in https://github.com/rising-turtle/LearnVIORB/tree/master 
**off_viorb.cpp** read rgb-d and imu data from disk. 
**off_viorb_dpt_first.cpp** use the depth data for the first frame. 

## VINS-Mono
The original VINS-Mono package is https://github.com/HKUST-Aerial-Robotics/VINS-Mono
Add launch and config file to run VINS-Mono using our dataset. 
Also, **vins-mapping.cpp** use the result of VINS-Mono and the camera's data (rgb-d) to generate trajectory.ply and map.ply. 

## DSO
The original dso_ros package is https://github.com/JakobEngel/dso_ros
The original dso package is https://github.com/JakobEngel/dso
compile it in ROS, plus add config and launch file to run it using our dataset. 
**main.cpp** dso_live
**main_dso_pangolin.cpp** dso_offline 

## graph
A simple gtsam based visual-inertial SLAM implementation, following the paper 
* **Information fusion in navigation systems via factor graph based incremental smoothing**, 
V. Indelman, S. Williams, M. Kaess, and F. Dellaert, Robotics and Autonomous Systems, vol. 61, no. 8, pp. 721–738, 2013

## imu_img_show 
A simple tool to publish the angular change (delta_roll,pitch,yaw) from imu's gyro and meanwhile publish camera's image. 
Another package can receive the published data and, show the gyro's rotation and meanwhile display the camera data.
https://github.com/rising-turtle/gyro_display
In this way, we can visually see whether and how the data from IMU and camera have been synchronized. 

## imu_pub
A simple IMU reader and pulisher. And a simple test to compute the gravity direction using the first several IMU readings. 

## msf-multi sensor fusion
The original MSF package is https://github.com/ethz-asl/ethzasl_msf 
Add some launch files to use SVO as vision-only part in MSF. 










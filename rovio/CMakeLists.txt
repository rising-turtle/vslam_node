
project(vslam_rovio)

cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w")

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
  roscpp
  rosbag
  nav_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  tf
  )

# Describe catkin Project
catkin_package(
#  DEPENDS Eigen OpenCV Sophus Boost
  CATKIN_DEPENDS roscpp rosbag nav_msgs std_msgs sensor_msgs cv_bridge tf
#  INCLUDE_DIRS include
#  LIBRARIES svo_visualizer
)

# boost
find_package(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) 

add_executable(traj_bag_to_log src/traj_bag_to_log.cpp)
target_link_libraries(traj_bag_to_log ${catkin_LIBRARIES})


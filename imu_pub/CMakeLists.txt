cmake_minimum_required(VERSION 2.8.3)
project(imu_pub)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  tf
  std_msgs
  sensor_msgs
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package( 
  CATKIN_DEPENDS roscpp tf std_msgs
  )

set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++11 -fpermissive") # -fPIC -fpermissive
include_directories(/usr/include ${catkin_INCLUDE_DIRS})  

#############################
# Eigen ####################
#############################
set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(${EIGEN_INCLUDE_DIRS})

# add_subdirectory(gtsam)

add_library(imu_pub src/imu_pub.cpp)
target_link_libraries(imu_pub ${catkin_LIBRARIES})




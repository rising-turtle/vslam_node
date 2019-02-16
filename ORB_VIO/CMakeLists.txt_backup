set(project_name vslam_viorb)
project(${project_name})
cmake_minimum_required(VERSION 2.4.6)
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# rosbuild_init()

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w  -O3 -march=native")

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

# LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
  roscpp
  rosbag
  nav_msgs
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf
)

# Describe catkin Project
catkin_package(
  # DEPENDS Eigen OpenCV Sophus Boost
  CATKIN_DEPENDS roscpp nav_msgs std_msgs sensor_msgs image_transport cv_bridge tf 
  # INCLUDE_DIRS include
  #  LIBRARIES svo_visualizer
)

find_package(OpenCV 2.4.3 REQUIRED)
# find_package(Eigen3 3.1.0 REQUIRED)
include_directories("/usr/include/eigen3")
find_package(Pangolin REQUIRED)

# relate to the previous project 
# set(PROJECT_SOURCE_DIR "/home/davidz/work/3rdlibs/LearnVIORB/Examples/ROS/ORB_VIO")
set(PROJECT_SOURCE_DIR "/home/davidz/work/github/LearnVIORB/Examples/ROS/ORB_VIO")

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../src
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
)

# Node for monocular camera
# rosbuild_add_executable(VIO src/ros_vio.cc src/MsgSync/MsgSynchronizer.cpp src/MsgSync/MsgSynchronizer.h)
add_executable(VIO src/ros_vio.cc src/MsgSync/MsgSynchronizer.cpp src/MsgSync/MsgSynchronizer.h)

target_link_libraries(VIO
${LIBS} ${catkin_LIBRARIES}
)

####### VIORB wrapper to handle our datasets
add_executable(viorb src/off_viorb.cpp src/imu_vn100.cpp src/MsgSync/MsgSynchronizer.cpp src/MsgSync/MsgSynchronizer.h)
target_link_libraries(viorb ${LIBS} ${catkin_LIBRARIES})

####### VIORB uses first dpt data to compute scale 
# add_executable(viorb_dpt src/off_viorb_dpt_first.cpp src/imu_vn100.cpp src/MsgSync/MsgSynchronizer.cpp src/MsgSync/MsgSynchronizer.h)
# target_link_libraries(viorb_dpt ${LIBS} ${catkin_LIBRARIES})

###### show ORB features 
add_executable(orb_feature src/orb_feature.cpp)
target_link_libraries(orb_feature ${LIBS} ${catkin_LIBRARIES})

###### test ORB match #####
add_executable(orb_match src/orb_match.cpp src/orb_matcher.cpp src/frame.cpp)
target_link_libraries(orb_match ${LIBS})

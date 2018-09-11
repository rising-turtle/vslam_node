project(vslam_vins)

cmake_minimum_required(VERSION 2.8.3)

SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
  roscpp
  # cmake_modules
  nav_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  tf
  )

# Describe catkin Project
catkin_package(
#  DEPENDS Eigen OpenCV Sophus Boost
  CATKIN_DEPENDS roscpp nav_msgs std_msgs sensor_msgs cv_bridge tf
#  INCLUDE_DIRS include
#  LIBRARIES svo_visualizer
)

# require OpenCV
find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}) 

# boost
find_package(Boost COMPONENTS filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS}) 

add_subdirectory(src bin)






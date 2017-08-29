SET(PROJECT_NAME vslam_imu_img_show)
PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED (VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
SET(CMAKE_VERBOSE_MAKEFILE OFF)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/CMakeModules/")

IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w -fpermissive")

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
  roscpp
  cmake_modules
  std_msgs
  cv_bridge
)

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)

# Describe catkin Project
catkin_package(
  DEPENDS Eigen OpenCV Sophus Boost
  CATKIN_DEPENDS roscpp cmake_modules std_msgs cv_bridge 
  INCLUDE_DIRS 
)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  src 
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(imu_img_show src/imu_img_show.cpp)
target_link_libraries(imu_img_show ${LINK_LIBS})


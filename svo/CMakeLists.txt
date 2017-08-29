SET(PROJECT_NAME vslam_svo)
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
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
  image_transport
  cv_bridge
  tf
  svo
  svo_msgs
  vikit_common
  vikit_ros
  graph_wrapper
)

# Add plain cmake packages 
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Eigen REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread)

# Describe catkin Project
catkin_package(
  DEPENDS Eigen OpenCV Sophus Boost
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs visualization_msgs sensor_msgs image_transport cv_bridge tf svo svo_msgs vikit_common vikit_ros
  INCLUDE_DIRS include
  LIBRARIES svo_visualizer
)

# Include dirs
INCLUDE_DIRECTORIES(
  include
  ${Eigen_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  src 
  ../graph/gtsam
  ../../graph_slam/gtsam 
  ../imu_pub/src
)

# Set link libraries
LIST(APPEND LINK_LIBS 
  ${OpenCV_LIBS}
  ${Sophus_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  gtsam_wrapper
  imu_interface
  imu_pub
)

################### GTSAM 4.0 ######################
# set(GTSAM_ROOT /home/david/work/3rdlibs/gtsam-4.0/build)
set(GTSAM_ROOT /home/davidz/work/3rdlibs/gtsam-4.0/build)
 find_package(GTSAM REQUIRED 
   PATHS ${GTSAM_ROOT}) 
include_directories(${GTSAM_INCLUDE_DIR} ${GTSAM_ROOT}) # ${GTSAM_ROOT}/../gtsam/3rdparty/Eigen) # /usr/include/eigen3
# link_directories(${GTSAM_ROOT}/gtsam )

# Create visualizer library
ADD_LIBRARY(vslam_svo_visualizer src/visualizer.cpp)
TARGET_LINK_LIBRARIES(vslam_svo_visualizer ${LINK_LIBS})
                                 
# Create Executables                      
ADD_EXECUTABLE(vslam_svo src/vo_node.cpp)
TARGET_LINK_LIBRARIES(vslam_svo vslam_svo_visualizer)

ADD_EXECUTABLE(vslam_svo_dpt src/vo_node_dpt.cpp)
TARGET_LINK_LIBRARIES(vslam_svo_dpt vslam_svo_visualizer) 

ADD_EXECUTABLE(fast_detector src/feature_detect.cpp)
TARGET_LINK_LIBRARIES(fast_detector ${LINK_LIBS})
# ADD_EXECUTABLE(benchmark src/benchmark_node.cpp)
# TARGET_LINK_LIBRARIES(benchmark svo_visualizer)

# compare good feature detector 
add_executable(good_feature src/good_feature_for_track.cpp)
target_link_libraries(good_feature ${LINK_LIBS})

# vio : svo + imu gtsam
add_executable(vslam_vio_svo src/vio_node_dpt.cpp)
target_link_libraries(vslam_vio_svo vslam_svo_visualizer) 

add_executable(vslam_svo_pipeline src/test_pipeline_visual.cpp )
target_link_libraries(vslam_svo_pipeline vslam_svo_visualizer)

# msckf : svo + imu msckf
add_executable(vslam_msckf_svo_dpt src/msckf_node_dpt.cpp)
target_link_libraries(vslam_msckf_svo_dpt ${LINK_LIBS} imu_pub vslam_svo_visualizer)

add_executable(vslam_msckf_svo src/msckf_node.cpp)
target_link_libraries(vslam_msckf_svo ${LINK_LIBS} imu_pub vslam_svo_visualizer)


SET(PROJECT_NAME vslam_okvis)
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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
  roscpp
  cmake_modules
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

# set(OKVIS_INSTALLATION <path/to/install>) # point to installation
set(OKVIS_INSTALLATION "/home/davidz/work/3rdlibs/okvis/build")

# require OpenCV
find_package( OpenCV COMPONENTS core highgui imgproc features2d REQUIRED )
include_directories(BEFORE ${OpenCV_INCLUDE_DIRS}) 

# require okvis
find_package( okvis 1.1 REQUIRED)
include_directories(${OKVIS_INCLUDE_DIRS})

# require brisk
find_package( brisk 2 REQUIRED)
include_directories(${BRISK_INCLUDE_DIRS})

# require ceres
list(APPEND CMAKE_PREFIX_PATH ${OKVIS_INSTALLATION})
find_package( Ceres REQUIRED )
include_directories(${CERES_INCLUDE_DIRS}) 

# require OpenGV
find_package(opengv REQUIRED)

# VISensor, if available
list(APPEND CMAKE_MODULE_PATH ${OKVIS_INSTALLATION}/lib/CMake ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
message(STATUS "##################### CMAKE_MODULE_PATH: ${CMAKE_MODULE_PATH}" )
find_package(VISensor)
if(VISENSORDRIVER_FOUND)
  message(STATUS "Found libvisensor.")
else()
  message(STATUS "libvisensor not found")
endif()

  FIND_PACKAGE(Boost COMPONENTS filesystem system REQUIRED)
  include_directories(${Boost_INCLUDE_DIRS}) 
  # add_executable(okvis_app_synchronous okvis_apps/src/okvis_app_synchronous.cpp)
  add_executable(vslam_okvis src/okvis_ros_node.cpp)
  target_link_libraries(vslam_okvis
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv 
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_matcher
    okvis_frontend 
    okvis_multisensor_processing
    pthread 
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
    
add_executable(vslam_okvis_euroc src/okvis_app.cpp)
target_link_libraries(vslam_okvis_euroc
    okvis_util
    okvis_kinematics
    okvis_time
    okvis_cv 
    okvis_common
    okvis_ceres
    okvis_timing
    okvis_matcher
    okvis_frontend 
    okvis_multisensor_processing
    pthread 
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)

  if(${VISENSORDRIVER_FOUND})
    target_link_libraries(vslam_okvis ${VISensorDriver_LIBRARY})
    target_link_libraries(vslam_okvis_euroc ${VISensorDriver_LIBRARY})
  endif()




cmake_minimum_required(VERSION 2.4.6)
project(vslam_dso)

# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

# rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH bin)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

# Add catkin and required ROS packages
FIND_PACKAGE(catkin REQUIRED COMPONENTS 
  roscpp
  roslib
  sensor_msgs
  cv_bridge
)

# Describe catkin Project
catkin_package(
  # DEPENDS Eigen OpenCV Sophus Boost
  DEPENDS Boost
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
  # INCLUDE_DIRS include
  #  LIBRARIES svo_visualizer
)

# set(DSO_PATH $ENV{DSO_PATH})
set(DSO_PATH "/home/davidz/work/3rdlibs/dso")
message("\n\n---- LOOKING FOR DSO at\n\"${DSO_PATH}\"")


find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread) 
find_library(DSO_LIBRARY dso ${DSO_PATH}/build/lib)

# SSE flags
# rosbuild_check_for_sse()
set(CMAKE_CXX_FLAGS
   "${SSE_FLAGS} -O2 -g -std=c++0x -fno-omit-frame-pointer"
)

set(SOURCE_FILES         
  src/main.cpp
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${DSO_PATH}/src
  ${DSO_PATH}/thirdparty/Sophus
  ${OpenCV_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)  

# rosbuild_add_executable(dso_live src/main.cpp ${SOURCE_FILES})
add_executable(dso_live ${SOURCE_FILES})
target_link_libraries(dso_live ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})
# rosbuild_link_boost(dso_live thread)

add_executable(dso_offline src/main_dso_pangolin.cpp)
target_link_libraries(dso_offline ${DSO_LIBRARY} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${catkin_LIBRARIES})






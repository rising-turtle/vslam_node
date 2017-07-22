cmake_minimum_required(VERSION 2.8.3)
project(graph_wrapper)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  tf
  )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package( 
  CATKIN_DEPENDS roscpp tf 
  )

set(CMAKE_CXX_FLAGS "-ggdb -g -O3 -w -std=c++11 -fpermissive") # -fPIC -fpermissive
include_directories(/usr/include ${catkin_INCLUDE_DIRS})  

#############################
# Eigen ####################
#############################
set(EIGEN_INCLUDE_DIRS "/usr/include/eigen3")
include_directories(${EIGEN_INCLUDE_DIRS})
add_definitions(${EIGEN_DEFINITIONS})
# add_definitions(-DEIGEN_DONT_ALIGN)

add_subdirectory(gtsam)




project(vslam-vins)

################ PCL #################
# set(PCL_DIR "/home/david/work/3rdlibs/pcl_1.7.1_src/pcl-pcl-1.7.1/build")
# set(PCL_DIR "/home/david/work/pcl/build")
# find_package(PCL 1.7 REQUIRED common io sample_consensus segmentation visualization)
find_package(PCL REQUIRED common io sample_consensus segmentation visualization kdtree
  PATHS "/usr/local/share/pcl-1.8")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

add_executable(vins-mapping vins-mapping.cpp)
target_link_libraries(vins-mapping  ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(vins-mapping-d16 vins-mapping-d16.cpp)
target_link_libraries(vins-mapping-d16 ${OpenCV_LIBS} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})

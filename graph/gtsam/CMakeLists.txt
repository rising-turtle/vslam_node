
project(graph_wrapper)

################### GTSAM 4.0 ######################
# set(GTSAM_ROOT /home/david/work/3rdlibs/gtsam-4.0/build)
set(GTSAM_ROOT /home/davidz/work/3rdlibs/gtsam-4.0/build)
 find_package(GTSAM REQUIRED 
   PATHS ${GTSAM_ROOT}) 
include_directories(${GTSAM_INCLUDE_DIR} ${GTSAM_ROOT}) # ${GTSAM_ROOT}/../gtsam/3rdparty/Eigen) # /usr/include/eigen3
link_directories(${GTSAM_ROOT}/gtsam )

#################### compile ######################
add_library(gtsam_wrapper match_result.cpp graph_gtsam.cpp)
target_link_libraries(gtsam_wrapper gtsam ${catkin_LIBRARIES})








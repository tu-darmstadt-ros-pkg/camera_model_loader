cmake_minimum_required(VERSION 2.8.3)
project(camera_model_loader)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  aslam_cameras
  aslam_time
  roscpp
  sm_boost
  sm_eigen
  sm_kinematics
  sm_logging
  sm_opencv
  sm_property_tree
  image_transport
  cv_bridge
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES camera_model_loader
  CATKIN_DEPENDS aslam_cameras aslam_time roscpp sm_boost sm_eigen sm_kinematics sm_logging sm_opencv sm_property_tree image_transport cv_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/camera_model_loader.h
  src/camera_model_loader.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


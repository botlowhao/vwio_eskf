cmake_minimum_required(VERSION 2.8.3)
project(vwio_eskf)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  tf
)

find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cmake_modules roscpp sensor_msgs
  DEPENDS EIGEN3 
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)


# eskf_localization node
add_executable(eskf_localization src/eskf_localization.cpp)
target_link_libraries(eskf_localization ${catkin_LIBRARIES})



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
  message_generation
)

add_message_files(
  FILES
  WOFISData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)  # 添加这一行来查找Ceres Solver

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS cmake_modules roscpp sensor_msgs std_msgs message_runtime
  DEPENDS EIGEN3 CERES  # 将CERES添加到依赖项中
)

###########
## Build ##
###########

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}  # 添加Ceres Solver的包含路径
)

# eskf_localization node
add_executable(eskf_localization src/eskf_localization.cpp)
target_link_libraries(eskf_localization 
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}  # 链接Ceres Solver库
)

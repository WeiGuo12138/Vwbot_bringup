cmake_minimum_required(VERSION 2.8.3)
project(vwbot_bringup)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  cmake_modules
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vwbot_bringup
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf geometry_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(vwbot_bringup_node
  src/vwbot_bringup_node.cc
  src/BoostSerialCommunicator.cc
  src/VwbotSerialHardware.cc
  src/VwbotRosDriver.cc
  src/utils.cc)

target_link_libraries(vwbot_bringup_node
  ${catkin_LIBRARIES}
)



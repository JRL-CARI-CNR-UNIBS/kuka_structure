cmake_minimum_required(VERSION 3.0.2)
project(kuka_structure_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuka_structure_gazebo
#  CATKIN_DEPENDS gazebo_ros roscpp
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 3.0.2)
project(exploration_manager)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    tf2_ros
    tf2_geometry_msgs
    pcl_ros
    pcl_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/exploration_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 
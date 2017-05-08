cmake_minimum_required(VERSION 2.8.3)
project(diffeo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES teleoperacion_hiro
#  CATKIN_DEPENDS joy moveit_commander moveit_msgs roscpp rospy std_msgs
#  DEPENDS sy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

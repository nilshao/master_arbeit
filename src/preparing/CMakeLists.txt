cmake_minimum_required(VERSION 3.0.2)
project(preparing)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  message_generation
  sensor_msgs
  cv_bridge
  image_transport
  visualization_msgs
)

find_package(OpenCV REQUIRED)
catkin_python_setup()
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES preparing
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)




catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py scripts/Get_Pic.py scripts/Get_Pic_2.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


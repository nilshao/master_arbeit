cmake_minimum_required(VERSION 3.0.2)
project(pic_reader)

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
##################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES compute_cdist
#  CATKIN_DEPENDS roscpp visualization_msgs robot_model_helper compute_cdist
#  DEPENDS assimp Eigen mlpack
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS scripts/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/Get_Pic.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(ShowPic src/ShowPic.cpp)
target_link_libraries(ShowPic ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(FindMarker src/FindMarker.cpp)
target_link_libraries(FindMarker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(HomoTrans src/HomoTrans.cpp)
target_link_libraries(HomoTrans ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

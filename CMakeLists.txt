cmake_minimum_required(VERSION 2.8.3)
project(mrl_ipcamera)


find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge dynamic_reconfigure image_transport sensor_msgs)
find_package( OpenCV REQUIRED )
include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
catkin_package(
 INCLUDE_DIRS
  include
 CATKIN_DEPENDS
  roscpp cv_bridge dynamic_reconfigure image_transport sensor_msgs
 DEPENDS
  OpenCV
)
add_executable(mrl_ipcamera src/mrl_ipcamera.cpp include/mrl_ipcamera/mrl_ipcamera.hpp)
target_link_libraries(mrl_ipcamera ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

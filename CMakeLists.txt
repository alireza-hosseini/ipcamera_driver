cmake_minimum_required(VERSION 2.8.3)
project(mrl_ipcamera)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
)

find_package( OpenCV REQUIRED )
include_directories(include ${catkin_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} )
catkin_package(
 CATKIN_DEPENDS
  cv_bridge
  dynamic_reconfigure
  image_transport
  roscpp
  sensor_msgs
 DEPENDS
  OpenCV
)

add_executable(${PROJECT_NAME} include/${PROJECT_NAME}/${PROJECT_NAME}.hpp
  src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


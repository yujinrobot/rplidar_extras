cmake_minimum_required(VERSION 2.8.3)
project(rplidar_extras)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  sensor_msgs
  nodelet
  pluginlib
  message_filters
  diagnostic_msgs
  diagnostic_updater
  rplidar_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(INCLUDE_DIRS include
               LIBRARIES rplidar_nodelet
               CATKIN_DEPENDS roscpp
                              rosconsole
                              sensor_msgs
                              nodelet
                              pluginlib
                              message_filters
                              diagnostic_msgs
                              diagnostic_updater
                              rplidar_ros
)

add_library(rplidar_nodelet src/rplidar_nodelet.cpp)
target_link_libraries(rplidar_nodelet ${catkin_LIBRARIES})

add_executable(rplidar_node src/rplidar_node.cpp)
target_link_libraries(rplidar_node rplidar_nodelet ${catkin_LIBRARIES})

install(TARGETS rplidar_node
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rplidar_nodelet
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
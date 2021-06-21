cmake_minimum_required(VERSION 2.8.3)
project(tiago_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  robot_framework_msgs
  std_msgs
  hardware_interface
  pluginlib
  controller_interface
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs robot_framework_msgs hardware_interface pluginlib controller_interface
)

add_library(JointController
  SHARED
    src/JointController.cpp
)

target_include_directories(JointController
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(JointController ${catkin_LIBRARIES})

install(TARGETS JointController
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES JointController_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/include)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)


cmake_minimum_required(VERSION 3.0.2)
project(beginner_tutorial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS scripts/talker.py scripts/listener.py scripts/main.py scripts/camera_node.py scripts/vision_node.py scripts/movement_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY weights/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/weights
)

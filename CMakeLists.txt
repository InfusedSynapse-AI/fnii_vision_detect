cmake_minimum_required(VERSION 3.0.2)
project(fnii_vision_detect)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Pixel.msg
  Object.msg
  Objects.msg
  MultiStreamObjects.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  Detect.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs message_runtime
)

# catkin_install_python(PROGRAMS
#   scripts/node.py scripts/yolo_detect.py  scripts/get_point.py scripts/__init__.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

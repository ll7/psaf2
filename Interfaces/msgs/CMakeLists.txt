cmake_minimum_required(VERSION 2.8.3)
project(custom_carla_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(
  DIRECTORY msg
  FILES
  CommonroadMap.msg
  GlobalPathLanelets.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  UpdateLocalPath.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)

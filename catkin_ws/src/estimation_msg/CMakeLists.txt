cmake_minimum_required(VERSION 3.0.2)
project(as_state_estimation_msg)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  StateEstimation.msg
  StateEstimationStamped.msg
)

generate_messages(DEPENDENCIES 
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



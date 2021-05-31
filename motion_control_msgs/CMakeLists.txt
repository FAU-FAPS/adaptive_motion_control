cmake_minimum_required(VERSION 3.0.2)
project(motion_control_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  moveit_msgs
  std_msgs
  trajectory_msgs
)

add_message_files(FILES VelocityCommand.msg)

add_service_files(FILES GetMotionPlan.srv)

generate_messages(DEPENDENCIES
  geometry_msgs
  moveit_msgs
  std_msgs
  trajectory_msgs
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  message_runtime
  moveit_msgs
  std_msgs
  trajectory_msgs
)

cmake_minimum_required(VERSION 3.0.2)
project(moveit_interface)

# enable C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# check build type
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: No build type specified. Selecting 'Release' for best performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  motion_control_msgs
  moveit_ros_planning_interface
  roscpp
  std_srvs
)

# export catkin dependencies
catkin_package(
 CATKIN_DEPENDS
   message_runtime
   motion_control_msgs
   moveit_ros_planning_interface
   roscpp
   std_srvs
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(moveit_interface src/moveit_interface.cpp src/moveit_interface_node.cpp)
target_link_libraries(moveit_interface ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS moveit_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

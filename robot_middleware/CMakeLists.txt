cmake_minimum_required(VERSION 3.0.2)
project(robot_middleware)

# find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  control_toolbox
  geometry_msgs
  moveit_core
  moveit_ros_planning
  motion_control_msgs
  roscpp
  rosparam_shortcuts
  simple_message
  std_msgs
  tf2_eigen
)

# required definitions for simple_message
add_definitions(-DROS)
add_definitions(-DLINUXSOCKETS=1)

# find system dependencies
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# enable C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# for clang-tidy
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# check build type
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: No build type specified. Selecting 'Release' for best performance")
  set(CMAKE_BUILD_TYPE Release)
endif()

# compile-out debug messages in Release configuration
if (CMAKE_BUILD_TYPE STREQUAL "Release")
  message("${PROJECT_NAME}: Setting ROS console log level to INFO in 'Release' config (compile-out debug messages)")
  add_definitions(-DROSCONSOLE_MIN_SEVERITY=ROSCONSOLE_SEVERITY_INFO)
endif()

# export catkin dependencies
catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    control_msgs
    control_toolbox
    geometry_msgs
    message_runtime
    moveit_core
    moveit_ros_planning
    motion_control_msgs
    roscpp
    rosparam_shortcuts
    simple_message
    std_msgs
    tf2_eigen
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

file(GLOB_RECURSE SOURCES
  src/*.cpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(robot_middleware ${SOURCES})
add_dependencies(robot_middleware ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_middleware
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  Threads::Threads
  simple_message
)

# byte swapping version
add_executable(robot_middleware_bswap ${SOURCES})
add_dependencies(robot_middleware_bswap ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_middleware_bswap
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  Threads::Threads
  simple_message_bswap
)

#############
## Install ##
#############

install(TARGETS
  robot_middleware
  robot_middleware_bswap
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

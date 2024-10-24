cmake_minimum_required(VERSION 3.0.2)
project(fcu_core)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
 find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  message_generation
  image_transport
  tf
  serial
  quadrotor_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  serial
  quadrotor_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}})

add_executable(fcu_bridge_001 src/fcu_bridge_001.cpp)
target_link_libraries(fcu_bridge_001 ${catkin_LIBRARIES} )

add_executable(fcu_command src/fcu_command.cpp)
target_link_libraries(fcu_command ${catkin_LIBRARIES} )

add_executable(fcu_mission src/fcu_mission.cpp)
target_link_libraries(fcu_mission ${catkin_LIBRARIES} )

add_executable(fcu_bridge_002 src/fcu_bridge_002.cpp)
target_link_libraries(fcu_bridge_002 ${catkin_LIBRARIES} )

add_executable(fcu_bridge_003 src/fcu_bridge_003.cpp)
target_link_libraries(fcu_bridge_003 ${catkin_LIBRARIES} )

add_executable(fcu_bridge_004 src/fcu_bridge_004.cpp)
target_link_libraries(fcu_bridge_004 ${catkin_LIBRARIES} )

add_executable(fcu_bridge_005 src/fcu_bridge_005.cpp)
target_link_libraries(fcu_bridge_005 ${catkin_LIBRARIES} )

add_executable(fcu_bridge_006 src/fcu_bridge_006.cpp)
target_link_libraries(fcu_bridge_006 ${catkin_LIBRARIES} )

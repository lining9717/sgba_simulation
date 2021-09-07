cmake_minimum_required(VERSION 3.0.2)
project(sgba_simulation)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -lpthread")


find_package(
  catkin REQUIRED
  COMPONENTS hector_gazebo_worlds
             hector_quadrotor_gazebo
             roscpp
             geometry_msgs
             nav_msgs
             message_generation
             tf
             std_msgs
             visualization_msgs
             roslib)

add_service_files(FILES EnableMotors.srv TrackingUAVId.srv)

add_message_files(FILES EntrancePosition.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  hector_gazebo_worlds
  hector_quadrotor_gazebo
  geometry_msgs
  roscpp
  std_msgs
  roslib)

include_directories(include ${catkin_INCLUDE_DIRS})

aux_source_directory(src/. DIR_SRCS)
add_executable(SGBASimulation src/sgba_simulation.cpp src/sgba_uav.cpp src/sgba_back_planner.cpp
src/sgba_bug_planner.cpp src/sgba_bug_main.cpp src/sgba_bug_grid.cpp)
target_link_libraries(SGBASimulation ${catkin_LIBRARIES})
add_dependencies(SGBASimulation ${catkin_EXPORTED_TARGETS})

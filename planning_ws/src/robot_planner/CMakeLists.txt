cmake_minimum_required(VERSION 3.0.2)
project(robot_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
cv_bridge
message_filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES build_map
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_executable(
  planner
  src/planner.cpp
  src/Astar.cpp
)

target_link_libraries(
        planner
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_executable(
  env1
  src/env1.cpp
  src/Astar.cpp
)

target_link_libraries(
        env1
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_executable(
  env2
  src/env2.cpp
  src/Astar.cpp
)

target_link_libraries(
        env2
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_executable(
    robot
    src/robot.cpp
    src/Astar.cpp
)

target_link_libraries(
        robot
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_executable(
    record
    src/record.cpp
)

target_link_libraries(
        record
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

add_executable(
    new
    src/new.cpp
    src/myastar.cpp
)

target_link_libraries(
        new
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)
cmake_minimum_required(VERSION 3.0.2)
project(occ_map)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

find_package(OpenCV REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  message_filters
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES occ_map
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

link_directories(${PCL_LIBRARY_DIRS})


add_executable(
   occ_map1
   src/occmap.cpp
   src/Astar.cpp
)

target_link_libraries(
        occ_map1
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)



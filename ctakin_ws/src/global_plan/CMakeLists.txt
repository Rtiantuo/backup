cmake_minimum_required(VERSION 3.0.2)
project(global_plan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(OpenCV REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES global_plan
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

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
        test_path 
        src/global_path.cpp
        src/astar.cpp
)
target_link_libraries(
        test_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_executable(
   occ_map
   src/occmap.cpp
   src/Astar.cpp
)

target_link_libraries(
        occ_map
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)


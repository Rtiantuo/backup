cmake_minimum_required(VERSION 3.0.2)
project(agg_plan)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories("/usr/include/eigen3")
include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES agg_plan
  # 上面这句在别的包引用这个包的头文件时候需要注释掉
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

add_executable(
      aggplan 
      src/aggplan.cpp
      src/astar3d.cpp)

add_library(trajoptim_shared SHARED src/trajoptim.cpp)
target_link_libraries(
        aggplan
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        trajoptim_shared
)

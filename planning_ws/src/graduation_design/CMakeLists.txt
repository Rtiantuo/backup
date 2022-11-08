cmake_minimum_required(VERSION 3.0.2)
project(graduation_design)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
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
  LIBRARIES graduation_design
  CATKIN_DEPENDS roscpp rospy std_msgs pcl_ros
#  DEPENDS system_lib
)


include_directories(
  include
  include/graduation_design
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")

add_executable (trajectory_generation 
                src/trajectory_generation.cpp
                src/PolyClosedForm_MinimumSnap.cpp)
target_link_libraries (trajectory_generation ${catkin_LIBRARIES} ${PCL_LIBRARIES})



add_executable (pathsearch
                src/pathsearch.cpp
                src/rrt.cpp)
target_link_libraries (pathsearch ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable (bezier_predict
                src/bezier_predict)
target_link_libraries (bezier_predict ${catkin_LIBRARIES} ${PCL_LIBRARIES})


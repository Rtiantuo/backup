cmake_minimum_required(VERSION 3.0.2)
project(msg_subscriber)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  cv_bridge
  image_transport
  pcl_conversions
  pcl_ros
)
find_package(PCL 1.3 REQUIRED)
include_directories("/usr/include/eigen3")
 
include_directories(${PCL_INCLUDE_DIRS})
 
link_directories(${PCL_LIBRARY_DIRS})
 
add_definitions(${PCL_DEFINITIONS})
 
 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msg_subscriber
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


include_directories(${catkin_INCLUDE_DIRS})


add_executable(msg_subscriber src/msg_subscriber.cpp)

add_executable(ros_pcl src/ros_pcl.cpp)

target_link_libraries(msg_subscriber
  ${catkin_LIBRARIES}
)
target_link_libraries(ros_pcl
  ${catkin_LIBRARIES}
)
target_link_libraries ( ros_pcl ${PCL_LIBRARIES})




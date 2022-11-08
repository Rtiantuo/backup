cmake_minimum_required(VERSION 3.0.2)
project(build_map)

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
  LIBRARIES build_map
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
   mymap
   src/mymap.cpp
   src/Astar.cpp
)

target_link_libraries(
        mymap
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
)

# add_executable(
#    nav
#    src/nav.cpp
#    src/Astar.cpp
# )

# target_link_libraries(
#         nav
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
# )

# add_executable(
#    pathfind
#    src/pathfind.cpp
#    src/Astar.cpp
# )

# target_link_libraries(
#         mymap
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
# )

# target_link_libraries(
#         pathfind
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
# )

# add_executable(
#    points
#    src/points.cpp
# )

# target_link_libraries(
#         points
#         ${catkin_LIBRARIES}
#         ${PCL_LIBRARIES}
# )

# add_executable(
#    imu
#    src/imu.cpp
# )

# target_link_libraries(
#         imu
#         ${catkin_LIBRARIES}
# )

# add_executable(
#    path_follow
#    src/path_follow.cpp
# )

# target_link_libraries(
#         path_follow
#         ${catkin_LIBRARIES}
# )
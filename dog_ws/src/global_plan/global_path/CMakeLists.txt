cmake_minimum_required(VERSION 3.0.2)
project(global_path)

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

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
include_directories(${JSONCPP_INCLUDE_DIRS})
link_libraries(${JSONCPP_LIBRARIES})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES global_path
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
        src/test_global_path.cpp
        src/test_astar.cpp
)
target_link_libraries(
        test_path
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

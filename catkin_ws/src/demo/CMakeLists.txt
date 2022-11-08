 cmake_minimum_required(VERSION 3.0.2)
project(demo)


find_package(catkin REQUIRED COMPONENTS
     	roscpp
        rospy
        std_msgs
        nav_msgs
        visualization_msgs
        tf
        cv_bridge
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV REQUIRED)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})


catkin_package(
        INCLUDE_DIRS include
)


include_directories(${PROJECT_SOURCE_DIR}/include)

include_directories(
        include
        SYSTEM
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

add_executable(emit src/emit.cpp)
add_executable(receive src/receive.cpp)

target_link_libraries(
        emit
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
)

target_link_libraries(
        receive
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
)


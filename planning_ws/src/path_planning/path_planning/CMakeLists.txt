cmake_minimum_required(VERSION 2.8.3)
project(path_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES path_planning
  CATKIN_DEPENDS roscpp std_msgs visualization_msgs
  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


## Declare a cpp library
add_library(path_planning
	src/rrt.cpp
	src/obstacles.cpp
)

add_executable(rrt_node src/rrt_node.cpp)
add_dependencies(rrt_node path_planning)
target_link_libraries(rrt_node path_planning ${catkin_LIBRARIES} )

add_executable(env_node src/environment.cpp)
add_dependencies(env_node path_planning)
target_link_libraries(env_node path_planning ${catkin_LIBRARIES} )

add_executable(RRT 
            src/RRT_node.cpp
            src/RRT.cpp)
target_link_libraries(RRT  ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
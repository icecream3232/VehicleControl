cmake_minimum_required(VERSION 3.0.2)
project(decision)

# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  common_msgs
  geometry_msgs
  roscpp
  std_msgs
  teb_local_planner
)

catkin_package(
#  CATKIN_DEPENDS common_msgs geometry_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)


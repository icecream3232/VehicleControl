cmake_minimum_required(VERSION 3.0.2)
project(v2i)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common_msgs
  roscpp
  std_msgs
  geometry_msgs
)


catkin_package(
#  CATKIN_DEPENDS common_msgs roscpp std_msgs geometry_msgs
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

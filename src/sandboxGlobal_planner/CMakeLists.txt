cmake_minimum_required(VERSION 3.0.2)
project(sandboxGlobal_planner)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    common_msgs
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES carrot_planner
  CATKIN_DEPENDS
    angles
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
    common_msgs
)

add_library(sandboxGlobal_planner src/sandboxGlobal_planner.cpp)
add_dependencies(sandboxGlobal_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sandboxGlobal_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS sandboxGlobal_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



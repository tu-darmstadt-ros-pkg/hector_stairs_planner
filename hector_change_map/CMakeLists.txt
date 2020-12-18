cmake_minimum_required(VERSION 3.0.2)
project(hector_change_map VERSION 0.0.0)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  map_server
  hector_change_layer_msgs
  dynamic_reconfigure
  tf2_ros
  tf2
  tf2_geometry_msgs
  hector_move_base_msgs
  grid_map_core
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(
  cfg/HectorChangeMap.cfg
)

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)
if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hector_change_map
  CATKIN_DEPENDS dynamic_reconfigure grid_map_core hector_change_layer_msgs map_server roscpp tf tf2_ros tf2 tf2_geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp library
add_library(${PROJECT_NAME}
  src/hector_change_map.cpp
  include/${PROJECT_NAME}/hector_change_map.h
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  yaml-cpp
  ${catkin_LIBRARIES}
)

## Declare a cpp executable
add_executable(hector_change_map_node src/hector_change_map_node.cpp)
add_dependencies(hector_change_map_node ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(hector_change_map_node ${PROJECT_NAME})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS ${PROJECT_NAME} hector_change_map_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


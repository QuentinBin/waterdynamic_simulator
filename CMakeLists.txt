cmake_minimum_required(VERSION 3.0.2)
project(waterdynamic_simulator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_dev
  roscpp
  rospy
  std_msgs
)

find_package(Eigen3 REQUIRED)
set(ROBODACT_GAZEBO_PLUGINS_LIST "")
# sdfformat9 uses C++17 features. 

add_definitions(-std=c++17)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES 
  underwater_object_plugin
  underwater_fin_plugin
 CATKIN_DEPENDS roscpp rospy
 DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${EIGEN3_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(underwater_object_plugin
  src/WaterDynamic.cc
  src/WaterDynamicPlugin.cc
)
list(APPEND ROBODACT_GAZEBO_PLUGINS_LIST underwater_object_plugin)

add_library(underwater_fin_plugin
  src/WaterDynamic.cc
  src/FinPlugin.cc
)
list(APPEND ROBODACT_GAZEBO_PLUGINS_LIST underwater_fin_plugin)


install(TARGETS ${ROBODACT_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)

install(DIRECTORY include/
   DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN ".hh"
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_waterdynamic_simulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

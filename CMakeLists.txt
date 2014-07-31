cmake_minimum_required(VERSION 2.8.3)
project(flexforce_adapter)
add_definitions(-std=c++0x)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flexforce_adapter
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ifk src/ifk.cpp)
target_link_libraries(ifk flexforce_adapter ${catkin_LIBRARIES})

add_library(flexforce_adapter src/flexforce_adapter.cpp include/flexforce_adapter.h)
target_link_libraries(flexforce_adapter phidget21)
add_dependencies(flexforce_adapter flexforce_adapter_gencpp)

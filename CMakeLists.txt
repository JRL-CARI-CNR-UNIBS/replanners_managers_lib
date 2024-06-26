cmake_minimum_required(VERSION 3.16)
project(replanners_managers_lib)

if(NOT CMAKE_BUILD_TYPE)
  # set(CMAKE_BUILD_TYPE "Release")
  set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto) #-O3
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(replanners_lib REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  graph_display
  cnr_scene_manager_msgs
  moveit_collision_checker
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp graph_display moveit_collision_checker cnr_scene_manager_msgs
 DEPENDS replanners_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/replanners_managers_lib.cpp
# )

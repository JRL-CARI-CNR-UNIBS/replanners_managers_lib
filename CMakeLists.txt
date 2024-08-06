cmake_minimum_required(VERSION 3.16)
project(replanners_managers_lib)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  #set(CMAKE_BUILD_TYPE "Debug")
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
find_package(trajectories_processors_lib REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  graph_display
  cnr_scene_manager_msgs
  moveit_collision_checker
  jsk_rviz_plugins
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp graph_display moveit_collision_checker cnr_scene_manager_msgs jsk_rviz_plugins
  DEPENDS replanners_lib trajectories_processors_lib
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

add_library(${PROJECT_NAME} SHARED
  src/replanners_managers/replanner_manager_base.cpp
  src/replanners_managers/replanner_manager_DRRT.cpp
  src/replanners_managers/replanner_manager_anytimeDRRT.cpp
  src/replanners_managers/replanner_manager_DRRTStar.cpp
  src/replanners_managers/replanner_manager_MPRRT.cpp
  src/replanners_managers/replanner_manager_MARS.cpp
  )
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(${PROJECT_NAME} PUBLIC replanners_lib::replanners_lib trajectories_processors_lib::trajectories_processors_lib )
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} )

# Install
install(DIRECTORY include/openmore
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

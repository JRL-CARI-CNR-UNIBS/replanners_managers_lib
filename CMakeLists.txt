cmake_minimum_required(VERSION 3.16)
project(replanners_managers_lib)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
  #set(CMAKE_BUILD_TYPE "Debug")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  if(CMAKE_BUILD_TYPE MATCHES "Release")
    add_compile_options(-funroll-loops -Wall -Ofast -flto -O3) #-O3
  endif()
  if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_compile_options(-Wall -g -Og)
  endif()
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(replanners_lib REQUIRED)
find_package(trajectories_processors_lib REQUIRED)
find_package(graph_display REQUIRED)
find_package(cnr_scene_manager_msgs REQUIRED)
find_package(moveit_collision_checker REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)

###########
## Build ##
###########
add_library(${PROJECT_NAME} SHARED
  src/replanners_managers/replanner_manager_base.cpp
  src/replanners_managers/replanner_manager_DRRT.cpp
  src/replanners_managers/replanner_manager_anytimeDRRT.cpp
  src/replanners_managers/replanner_manager_DRRTStar.cpp
  src/replanners_managers/replanner_manager_MPRRT.cpp
  src/replanners_managers/replanner_manager_MARS.cpp
  )
target_link_libraries(${PROJECT_NAME} PUBLIC replanners_lib::replanners_lib trajectories_processors_lib::trajectories_processors_lib)
target_include_directories(${PROJECT_NAME} 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp graph_display cnr_scene_manager_msgs moveit_collision_checker rviz_2d_overlay_msgs)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET) # exports the targets for CMake, to allow your library’s clients to use the target_link_libraries(client my_library::my_library) syntax.
ament_export_dependencies(rclcpp graph_display cnr_scene_manager_msgs moveit_collision_checker replanners_lib trajectories_processors_lib rviz_2d_overlay_msgs)

# Install
install(TARGETS
        ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
    )

install(
      DIRECTORY include/openmore
      DESTINATION include
    )

# Clang-format target
file(GLOB_RECURSE ALL_SOURCE_FILES *.cpp *.h)

add_custom_target(
    format
    COMMAND chmod +x cmake_config/clang_format_target.sh
    COMMAND cmake_config/clang_format_target.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMENT "Running clang-format on all source files"
)

ament_package()

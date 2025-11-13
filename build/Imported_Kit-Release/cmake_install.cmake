# Install script for directory: /home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/environment" TYPE FILE FILES "/opt/ros/jazzy/lib/python3.12/site-packages/ament_package/template/environment_hook/library_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/environment" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/library_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreplanners_managers_lib.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreplanners_managers_lib.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreplanners_managers_lib.so"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/libreplanners_managers_lib.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreplanners_managers_lib.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreplanners_managers_lib.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreplanners_managers_lib.so"
         OLD_RPATH "/home/riemann/projects/openmore_ws/install/replanners_lib/lib:/home/riemann/projects/openmore_ws/install/trajectories_processors_lib/lib:/home/riemann/projects/openmore_ws/install/graph_display/lib:/home/riemann/projects/openmore_ws/install/cnr_scene_manager_msgs/lib:/home/riemann/projects/openmore_ws/install/moveit_collision_checker/lib:/opt/ros/jazzy/lib/x86_64-linux-gnu:/opt/ros/jazzy/lib:/opt/ros/jazzy/opt/rviz_ogre_vendor/lib:/home/riemann/projects/openmore_ws/install/graph_core/lib:/home/riemann/projects/openmore_ws/install/cnr_logger/lib:/home/riemann/projects/openmore_ws/install/cnr_param/lib:/home/riemann/projects/openmore_ws/install/cnr_yaml/lib:/home/riemann/projects/openmore_ws/install/cnr_class_loader/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libreplanners_managers_lib.so")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/include/openmore")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/replanners_managers_lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/replanners_managers_lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/environment" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/environment" TYPE FILE FILES "/opt/ros/jazzy/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/environment" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_index/share/ament_index/resource_index/packages/replanners_managers_lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake/replanners_managers_libTargetsExport.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake/replanners_managers_libTargetsExport.cmake"
         "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/CMakeFiles/Export/b7912868ec0d2f288d72c7810ad12f66/replanners_managers_libTargetsExport.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake/replanners_managers_libTargetsExport-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake/replanners_managers_libTargetsExport.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/CMakeFiles/Export/b7912868ec0d2f288d72c7810ad12f66/replanners_managers_libTargetsExport.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/CMakeFiles/Export/b7912868ec0d2f288d72c7810ad12f66/replanners_managers_libTargetsExport-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_export_targets/ament_cmake_export_targets-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_export_dependencies/ament_cmake_export_dependencies-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib/cmake" TYPE FILE FILES
    "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_core/replanners_managers_libConfig.cmake"
    "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/ament_cmake_core/replanners_managers_libConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/replanners_managers_lib" TYPE FILE FILES "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/riemann/projects/openmore_ws/src/OpenMORE/replanners_managers_lib/build/Imported_Kit-Release/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")

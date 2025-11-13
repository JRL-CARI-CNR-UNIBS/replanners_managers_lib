#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "replanners_managers_lib::replanners_managers_lib" for configuration "Release"
set_property(TARGET replanners_managers_lib::replanners_managers_lib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(replanners_managers_lib::replanners_managers_lib PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libreplanners_managers_lib.so"
  IMPORTED_SONAME_RELEASE "libreplanners_managers_lib.so"
  )

list(APPEND _cmake_import_check_targets replanners_managers_lib::replanners_managers_lib )
list(APPEND _cmake_import_check_files_for_replanners_managers_lib::replanners_managers_lib "${_IMPORT_PREFIX}/lib/libreplanners_managers_lib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

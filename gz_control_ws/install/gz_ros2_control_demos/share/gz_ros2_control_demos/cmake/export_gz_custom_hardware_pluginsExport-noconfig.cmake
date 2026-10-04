#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "gz_ros2_control_demos::gz_custom_hardware_plugins" for configuration ""
set_property(TARGET gz_ros2_control_demos::gz_custom_hardware_plugins APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(gz_ros2_control_demos::gz_custom_hardware_plugins PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libgz_custom_hardware_plugins.so"
  IMPORTED_SONAME_NOCONFIG "libgz_custom_hardware_plugins.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS gz_ros2_control_demos::gz_custom_hardware_plugins )
list(APPEND _IMPORT_CHECK_FILES_FOR_gz_ros2_control_demos::gz_custom_hardware_plugins "${_IMPORT_PREFIX}/lib/libgz_custom_hardware_plugins.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)

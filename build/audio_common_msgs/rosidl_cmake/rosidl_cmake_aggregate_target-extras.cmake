# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target audio_common_msgs::audio_common_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${audio_common_msgs_TARGETS}.
if(audio_common_msgs_TARGETS AND NOT TARGET audio_common_msgs::audio_common_msgs)
  add_library(audio_common_msgs::audio_common_msgs INTERFACE IMPORTED)
  set_target_properties(audio_common_msgs::audio_common_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${audio_common_msgs_TARGETS}")
endif()

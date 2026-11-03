cmake_minimum_required(VERSION 3.8)

project(quadruped_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS
  "src/*.cpp"
)

add_executable(
  quadruped
  ${SOURCES}
)

target_include_directories(
  quadruped
  PUBLIC
  include
)

ament_target_dependencies(
  quadruped
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  kdl_parser
  ament_index_cpp
)

target_link_libraries(
  quadruped
  orocos-kdl
  ${OpenCV_LIBS}
)

install(
  TARGETS quadruped
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
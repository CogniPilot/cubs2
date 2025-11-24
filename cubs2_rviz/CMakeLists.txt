cmake_minimum_required(VERSION 3.8)
project(cubs2_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cubs2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

include_directories(
  include
)

# RViz plugin library
add_library(cubs2_rviz SHARED
  src/rviz/joy_panel.cpp
  src/rviz/sim_panel.cpp
  src/rviz/hud_panel.cpp
)

ament_target_dependencies(cubs2_rviz
  rclcpp
  rviz_common
  pluginlib
  cubs2_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
)

target_link_libraries(cubs2_rviz Qt5::Widgets)

# Install library
install(TARGETS
  cubs2_rviz
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install plugin description
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  rviz_common
  pluginlib
  cubs2_msgs
  geometry_msgs
  std_msgs
  sensor_msgs
  Qt5
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.5)
project(yubot_purepursuit_controller)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav2_bringup REQUIRED)

nav2_package()

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  pluginlib
  nav_msgs
  nav2_util
  nav2_core
  tf2
  nav2_bringup
)

add_library(yubot_purepursuit_controller SHARED
        src/pure_controller.cpp)

# prevent pluginlib from using boost
target_compile_definitions(yubot_purepursuit_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(yubot_purepursuit_controller
  ${dependencies}
)

install(TARGETS yubot_purepursuit_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ 
  DESTINATION include/
)

install(
  DIRECTORY launch params map
  DESTINATION share/${PROJECT_NAME}
)
install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(yubot_purepursuit_controller)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

ament_package()


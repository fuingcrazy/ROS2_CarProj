set(_AMENT_PACKAGE_NAME "yubot_controller")
set(yubot_controller_VERSION "0.0.0")
set(yubot_controller_MAINTAINER "goblue <ygx20020926@gmail.com>")
set(yubot_controller_BUILD_DEPENDS "nav2_common" "nav2_core" "nav2_util" "nav2_costmap_2d" "rclcpp" "geometry_msgs" "nav2_msgs" "pluginlib" "tf2" "tf2_geometry_msgs" "nav2_bringup")
set(yubot_controller_BUILDTOOL_DEPENDS "ament_cmake")
set(yubot_controller_BUILD_EXPORT_DEPENDS "nav2_common" "nav2_core" "nav2_util" "nav2_costmap_2d" "rclcpp" "geometry_msgs" "nav2_msgs" "pluginlib" "tf2" "tf2_geometry_msgs" "nav2_bringup")
set(yubot_controller_BUILDTOOL_EXPORT_DEPENDS )
set(yubot_controller_EXEC_DEPENDS "nav2_common" "nav2_core" "nav2_util" "nav2_costmap_2d" "rclcpp" "geometry_msgs" "nav2_msgs" "pluginlib" "tf2" "tf2_geometry_msgs" "nav2_bringup")
set(yubot_controller_TEST_DEPENDS "ament_cmake_gtest" "ament_lint_common" "ament_lint_auto")
set(yubot_controller_GROUP_DEPENDS )
set(yubot_controller_MEMBER_OF_GROUPS )
set(yubot_controller_DEPRECATED "")
set(yubot_controller_EXPORT_TAGS)
list(APPEND yubot_controller_EXPORT_TAGS "<build_type>ament_cmake</build_type>")
list(APPEND yubot_controller_EXPORT_TAGS "<nav2_core plugin=\"plugin.xml\"/>")

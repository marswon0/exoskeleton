set(_AMENT_PACKAGE_NAME "exo_urdf")
set(exo_urdf_VERSION "0.0.0")
set(exo_urdf_MAINTAINER "bob <marswon0@gmail.com>")
set(exo_urdf_BUILD_DEPENDS "urdf" "hardware_interface" "gazebo_ros2_control" "backward_ros" "pluginlib" "rclcpp" "rclcpp_lifecycle")
set(exo_urdf_BUILDTOOL_DEPENDS "ament_cmake")
set(exo_urdf_BUILD_EXPORT_DEPENDS "urdf" "hardware_interface" "gazebo_ros2_control" "backward_ros" "pluginlib" "rclcpp" "rclcpp_lifecycle")
set(exo_urdf_BUILDTOOL_EXPORT_DEPENDS )
set(exo_urdf_EXEC_DEPENDS "rviz2" "xacro" "robot_state_publisher" "joint_state_broadcaster" "joint_state_publisher_gui" "gazebo_ros" "gazebo_ros_pkgs" "ros2controlcli" "ros2launch" "controller_manager" "urdf" "hardware_interface" "gazebo_ros2_control" "backward_ros" "pluginlib" "rclcpp" "rclcpp_lifecycle")
set(exo_urdf_TEST_DEPENDS "ament_lint_auto" "ament_lint_common" "ament_cmake_pytest" "launch_testing" "launch" "liburdfdom-tools" "rclpy")
set(exo_urdf_GROUP_DEPENDS )
set(exo_urdf_MEMBER_OF_GROUPS )
set(exo_urdf_DEPRECATED "")
set(exo_urdf_EXPORT_TAGS)
list(APPEND exo_urdf_EXPORT_TAGS "<build_type>ament_cmake</build_type>")

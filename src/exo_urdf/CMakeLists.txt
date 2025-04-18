cmake_minimum_required(VERSION 3.8)
project(exo_urdf)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)

# Install directories
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


# cmake_minimum_required(VERSION 3.8)
# project(exoskeleton_description)

# find_package(ament_cmake REQUIRED)
# find_package(robot_state_publisher REQUIRED)
# find_package(rviz2 REQUIRED)
# find_package(xacro REQUIRED)

# # Install URDF files
# install(
#   DIRECTORY urdf
#   DESTINATION share/${PROJECT_NAME}
# )

# # Install launch files
# install(
#   DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME}
# )

# # Install RViz config
# install(
#   DIRECTORY rviz
#   DESTINATION share/${PROJECT_NAME}
# )

# ament_package()


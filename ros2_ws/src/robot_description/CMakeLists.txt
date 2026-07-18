cmake_minimum_required(VERSION 3.5)
project(robot_description)

find_package(ament_cmake REQUIRED)

# Install URDF + launch into share/ so other packages (bringup) can find them.
install(
  DIRECTORY urdf launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

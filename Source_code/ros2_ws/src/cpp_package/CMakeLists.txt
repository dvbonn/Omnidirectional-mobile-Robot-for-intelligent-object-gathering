cmake_minimum_required(VERSION 3.5)
project(cpp_package)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(wifi_node src/wifi_node.cpp src/socket_handler.cpp src/mdns_service.cpp)
add_executable(explorer_node src/explorer_node.cpp src/frontier_based_exploration.cpp)

target_link_libraries(wifi_node
  "avahi-client"
  "avahi-common"
)

ament_target_dependencies(wifi_node rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros)
ament_target_dependencies(explorer_node rclcpp rclcpp_action nav_msgs nav2_msgs 
                          geometry_msgs tf2_ros tf2_geometry_msgs)

target_include_directories(wifi_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_include_directories(explorer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS wifi_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS explorer_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
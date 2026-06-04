cmake_minimum_required(VERSION 3.8)
project(multi_explore_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ==================== Find Dependencies ====================
# C++ コードが実際に #include / link するものだけ find_package する
# slam_toolbox, joint_state_publisher, robot_state_publisher 等は
# CMake config を export しないので package.xml の <depend> のみで十分
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ==================== Build C++ Executables ====================

# 1. Sensor Explore Node
add_executable(sensor_explore_node src/sensor_explore_node.cpp)
ament_target_dependencies(sensor_explore_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
)

# 2. Explore Supervisor
add_executable(explore_supervisor src/explore_supervisor.cpp)
ament_target_dependencies(explore_supervisor
  rclcpp
  nav_msgs
  geometry_msgs
  std_srvs
)

# ==================== Install ====================
install(TARGETS
  sensor_explore_node
  explore_supervisor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

# ==================== Testing ====================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
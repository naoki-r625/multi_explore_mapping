cmake_minimum_required(VERSION 3.8)
project(multi_explore_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 を指定（Eigen3 推奨）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ==================== Find Dependencies ====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ✅ ICP 実装に必要な依存を追加
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ==================== Include Directories ====================
include_directories(include)

# ==================== Build C++ Libraries ====================

# ✅ ICP ライブラリ
add_library(icp_lib
  src/icp.cpp
)

target_link_libraries(icp_lib
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
)

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

# ✅ 3. ICP Map Matching Node
add_executable(icp_map_matching_node
  src/icp_map_matching_node.cpp
)

target_link_libraries(icp_map_matching_node
  icp_lib
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
)

ament_target_dependencies(icp_map_matching_node
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  cv_bridge
)

# ==================== Install ====================
install(TARGETS
  sensor_explore_node
  explore_supervisor
  icp_map_matching_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
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
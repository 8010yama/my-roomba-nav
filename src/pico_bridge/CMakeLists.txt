cmake_minimum_required(VERSION 3.8)
project(pico_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Pico と ROS 2 をつなぐノード（実行ファイル名は Python 版と同じ「bridge」）
add_executable(bridge src/bridge_node.cpp)
target_include_directories(bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(bridge PUBLIC cxx_std_17)
ament_target_dependencies(bridge rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2_ros)

install(TARGETS bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_diffdrive test/test_diffdrive.cpp)
    target_compile_definitions(test_diffdrive PRIVATE USE_GTEST)
    target_include_directories(test_diffdrive PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
endif()

ament_package()

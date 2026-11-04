cmake_minimum_required(VERSION 3.8)
project(roomba_tasks)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)   # 経路の計算が速くなる
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 地図の上の仕事（ウェイポイント・くまなく掃除）
add_executable(task_manager src/task_manager.cpp)
target_include_directories(task_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(task_manager PUBLIC cxx_std_17)
ament_target_dependencies(task_manager rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs std_msgs std_srvs
  tf2 tf2_ros visualization_msgs)

# 自己位置推定・自動清掃・定位置に戻る（/localize・/clean・/set_home・/go_home）
add_executable(localize src/localize.cpp)
target_compile_features(localize PUBLIC cxx_std_17)
ament_target_dependencies(localize rclcpp rclcpp_action geometry_msgs nav2_msgs std_msgs std_srvs)

# コントローラのボタンでブラシ（R：メイン、ZR：サイド）
add_executable(joy_brush src/joy_brush.cpp)
target_compile_features(joy_brush PUBLIC cxx_std_17)
ament_target_dependencies(joy_brush rclcpp sensor_msgs std_msgs)

install(TARGETS task_manager localize joy_brush DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_coverage test/test_coverage.cpp)
    target_compile_definitions(test_coverage PRIVATE USE_GTEST)
    target_include_directories(test_coverage PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(laser_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# /scan → /scan_fixed（点の数をいつも同じにする）
add_executable(scan_normalizer src/scan_normalizer.cpp)
target_include_directories(scan_normalizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_compile_features(scan_normalizer PUBLIC cxx_std_17)
ament_target_dependencies(scan_normalizer rclcpp sensor_msgs)

install(TARGETS scan_normalizer DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest(test_resample test/test_resample.cpp)
    target_compile_definitions(test_resample PRIVATE USE_GTEST)
    target_include_directories(test_resample PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  endif()
endif()

ament_package()

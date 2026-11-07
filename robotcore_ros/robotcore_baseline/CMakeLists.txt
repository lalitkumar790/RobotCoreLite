cmake_minimum_required(VERSION 3.16)
project(robotcore_baseline)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dds_pingpong src/dds_pingpong.cpp)
ament_target_dependencies(dds_pingpong rclcpp std_msgs)
target_compile_options(dds_pingpong PRIVATE -Wall -Wextra -Wpedantic)

add_executable(dds_adapter_smoke src/dds_adapter_smoke.cpp)
target_include_directories(dds_adapter_smoke PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../robotcore_core/include)
ament_target_dependencies(dds_adapter_smoke rclcpp std_msgs)
target_compile_options(dds_adapter_smoke PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS dds_pingpong dds_adapter_smoke DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()

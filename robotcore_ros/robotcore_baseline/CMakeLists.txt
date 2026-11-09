cmake_minimum_required(VERSION 3.16)
project(robotcore_baseline)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(dds_pingpong src/dds_pingpong.cpp)
ament_target_dependencies(dds_pingpong rclcpp std_msgs)
target_compile_options(dds_pingpong PRIVATE -Wall -Wextra -Wpedantic)

add_executable(dds_adapter_smoke src/dds_adapter_smoke.cpp)
target_include_directories(dds_adapter_smoke PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../../robotcore_core/include)
ament_target_dependencies(dds_adapter_smoke rclcpp std_msgs)
target_compile_options(dds_adapter_smoke PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(transport_bench_dds ../../benchmarks/latency/transport_benchmark.cpp)
target_compile_definitions(transport_bench_dds PRIVATE ROBOTCORE_BENCH_DDS)
target_include_directories(transport_bench_dds PRIVATE include ../../robotcore_core/include)
ament_target_dependencies(transport_bench_dds rclcpp std_msgs)
target_compile_options(transport_bench_dds PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Native ROS 2 sensor_msgs/LaserScan -> RobotCore-Lite IPC -> independent sink.
# One source/bridge/sink executable with separate OS process roles.
add_library(robotcore_sensor_demo_ipc STATIC ../../robotcore_transports/ipc/src/ipc.cpp)
target_include_directories(robotcore_sensor_demo_ipc PUBLIC
  ../../robotcore_transports/ipc/include
  ../../robotcore_core/include)
target_compile_features(robotcore_sensor_demo_ipc PUBLIC cxx_std_17)
target_link_libraries(robotcore_sensor_demo_ipc PUBLIC Threads::Threads)
target_compile_options(robotcore_sensor_demo_ipc PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_executable(sensor_ipc_demo src/sensor_ipc_demo.cpp)
target_include_directories(sensor_ipc_demo PRIVATE include)
target_link_libraries(sensor_ipc_demo robotcore_sensor_demo_ipc Threads::Threads)
ament_target_dependencies(sensor_ipc_demo rclcpp sensor_msgs)
target_compile_options(sensor_ipc_demo PRIVATE -Wall -Wextra -Wpedantic -Werror)

# Execute actual ROS 2 CDR encode/decode tests with the Jazzy type support.
add_executable(scan_codec_test src/scan_codec_test.cpp)
target_include_directories(scan_codec_test PRIVATE include)
ament_target_dependencies(scan_codec_test rclcpp sensor_msgs)
target_compile_options(scan_codec_test PRIVATE -Wall -Wextra -Wpedantic -Werror)

install(TARGETS dds_pingpong dds_adapter_smoke transport_bench_dds
  sensor_ipc_demo scan_codec_test DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_package()

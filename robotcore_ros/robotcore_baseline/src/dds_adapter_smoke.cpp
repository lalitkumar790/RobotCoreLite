// Single-process functional smoke test for the Transport interface and DDS.
// Not a latency or cross-process benchmark.
#include "robotcore_ros/dds_transport.hpp"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int code = 1;
  {
    auto send_node = std::make_shared<rclcpp::Node>("robotcore_api_smoke_sender");
    auto recv_node = std::make_shared<rclcpp::Node>("robotcore_api_smoke_receiver");
    robotcore::Configuration config;
    config.max_payload_bytes = 256;
    robotcore_ros::DdsTransport sender(send_node, "robotcore/api_smoke", "robotcore/unused_sender", config);
    robotcore_ros::DdsTransport receiver(recv_node, "robotcore/unused_receiver", "robotcore/api_smoke", config);
    bool received = false;
    if (!sender.start([](robotcore::Message) {}) ||
        !receiver.start([&](robotcore::Message m) {
          if (m.sequence == 123 && m.payload == std::vector<std::uint8_t>{0, 1, 2, 255}) received = true;
        })) {
      std::cerr << "failed to start DDS adapter\n";
    } else {
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(send_node);
      executor.add_node(recv_node);
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
      // Retry during DDS discovery. This is a functional smoke test only.
      while (rclcpp::ok() && !received && std::chrono::steady_clock::now() < deadline) {
        const auto result = sender.publish({123, {0, 1, 2, 255}});
        if (!result) break;
        executor.spin_some(std::chrono::milliseconds(20));
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      code = received ? 0 : 1;
      if (!received) std::cerr << "DDS adapter smoke test did not receive expected message\n";
      sender.stop();
      receiver.stop();
    }
  }
  rclcpp::shutdown();
  return code;
}

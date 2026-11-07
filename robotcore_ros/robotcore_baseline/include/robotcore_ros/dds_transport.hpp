#pragma once

#include "robotcore/core.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace robotcore_ros {

// One DDS endpoint using a request topic and a reply topic, for cross-process
// or same-process use. Executor spin is owned by the ROS application.
// start()/stop()/publish() must run on the executor-management thread;
// don't stop concurrently with a user callback.
class DdsTransport final : public robotcore::Transport {
public:
  DdsTransport(rclcpp::Node::SharedPtr node, std::string send_topic,
               std::string receive_topic, robotcore::Configuration config)
    : node_(std::move(node)), send_topic_(std::move(send_topic)),
      receive_topic_(std::move(receive_topic)), config_(config) {}

  robotcore::TransportKind kind() const noexcept override {
    return robotcore::TransportKind::dds;
  }

  robotcore::Status start(robotcore::ReceiveCallback callback) override {
    if (started_) return robotcore::Status::failure(robotcore::Error::already_started, "already started");
    if (!node_ || send_topic_.empty() || receive_topic_.empty() || !callback)
      return robotcore::Status::failure(robotcore::Error::invalid_argument, "node, topics and callback required");
    const auto choice = robotcore::resolve_transport(config_);
    if (!choice.first) return choice.first;
    if (choice.second != robotcore::TransportKind::dds)
      return robotcore::Status::failure(robotcore::Error::unsupported, "DDS adapter requires DDS resolution");
    const auto qos = config_.reliable
      ? rclcpp::QoS(rclcpp::KeepLast(config_.queue_depth)).reliable()
      : rclcpp::QoS(rclcpp::KeepLast(config_.queue_depth)).best_effort();
    try {
      publisher_ = node_->create_publisher<std_msgs::msg::UInt8MultiArray>(send_topic_, qos);
      callback_ = std::move(callback);
      subscriber_ = node_->create_subscription<std_msgs::msg::UInt8MultiArray>(
        receive_topic_, qos, [this](std_msgs::msg::UInt8MultiArray::ConstSharedPtr packet) {
          if (packet->data.size() < 8 || packet->data.size() - 8 > config_.max_payload_bytes) return;
          robotcore::Message received;
          for (unsigned i = 0; i < 8; ++i)
            received.sequence |= static_cast<std::uint64_t>(packet->data[i]) << (8 * i);
          received.payload.assign(packet->data.begin() + 8, packet->data.end());
          if (callback_) callback_(std::move(received));
        });
      started_ = true;
      return robotcore::Status::success();
    } catch (const std::exception& e) {
      subscriber_.reset();
      publisher_.reset();
      callback_ = {};
      return robotcore::Status::failure(robotcore::Error::io_error, e.what());
    }
  }

  robotcore::Status publish(const robotcore::Message& message) override {
    if (!started_) return robotcore::Status::failure(robotcore::Error::not_started, "not started");
    const auto check = robotcore::validate_message(message, config_);
    if (!check) return check;
    std_msgs::msg::UInt8MultiArray packet;
    packet.data.reserve(8 + message.payload.size());
    for (unsigned i = 0; i < 8; ++i)
      packet.data.push_back(static_cast<std::uint8_t>((message.sequence >> (8 * i)) & 0xff));
    packet.data.insert(packet.data.end(), message.payload.begin(), message.payload.end());
    try {
      publisher_->publish(packet);
      return robotcore::Status::success();
    } catch (const std::exception& e) {
      return robotcore::Status::failure(robotcore::Error::io_error, e.what());
    }
  }

  robotcore::Status stop() override {
    subscriber_.reset();
    publisher_.reset();
    callback_ = {};
    started_ = false;
    return robotcore::Status::success();
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string send_topic_;
  std::string receive_topic_;
  robotcore::Configuration config_;
  robotcore::ReceiveCallback callback_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr subscriber_;
  bool started_{false};
};

} // namespace robotcore_ros

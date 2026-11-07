// RobotCore-Lite Phase 1: ROS 2 DDS round-trip baseline.
// Uses steady_clock on one process; RTT/2 is NOT a one-way measurement.
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using Packet = std_msgs::msg::UInt8MultiArray;
using Clock = std::chrono::steady_clock;

namespace {
void put_sequence(std::vector<uint8_t>& data, uint64_t seq) {
  for (unsigned i = 0; i < 8; ++i) data[i] = static_cast<uint8_t>((seq >> (8 * i)) & 0xff);
}
uint64_t get_sequence(const std::vector<uint8_t>& data) {
  uint64_t seq = 0;
  for (unsigned i = 0; i < 8; ++i) seq |= static_cast<uint64_t>(data[i]) << (8 * i);
  return seq;
}
std::string csv_escape(const std::string& x) {
  std::string y = """;
  for (char c : x) { if (c == '"') y += "\"\""; else y += c; }
  return y + """;
}
}

class Responder final : public rclcpp::Node {
public:
  explicit Responder(size_t payload, const rclcpp::QoS& qos)
  : Node("robotcore_dds_responder"), payload_(payload) {
    publisher_ = create_publisher<Packet>("robotcore/echo", qos);
    subscriber_ = create_subscription<Packet>("robotcore/request", qos,
      [this](Packet::ConstSharedPtr msg) {
        if (msg->data.size() != payload_) return;
        publisher_->publish(*msg);
      });
  }
private:
  size_t payload_;
  rclcpp::Publisher<Packet>::SharedPtr publisher_;
  rclcpp::Subscription<Packet>::SharedPtr subscriber_;
};

class Initiator final : public rclcpp::Node {
public:
  Initiator(size_t payload, uint64_t samples, uint64_t warmup,
            std::chrono::milliseconds timeout, const std::string& csv,
            const std::string& run_id, const rclcpp::QoS& qos)
  : Node("robotcore_dds_initiator"), payload_(payload), target_(samples),
    warmup_(warmup), timeout_(timeout), run_id_(run_id), data_(payload, 0xa5),
    output_(csv, std::ios::out | std::ios::trunc) {
    if (!output_) throw std::runtime_error("cannot open CSV: " + csv);
    output_ << "run_id,seq,payload_bytes,status,rtt_us,rtt_half_proxy_us\n";
    publisher_ = create_publisher<Packet>("robotcore/request", qos);
    subscriber_ = create_subscription<Packet>("robotcore/echo", qos,
      [this](Packet::ConstSharedPtr msg) { receive(*msg); });
    tick_ = create_wall_timer(1ms, [this] { advance(); });
  }
  bool done() const { return finished_; }
  uint64_t lost() const { return lost_; }
  uint64_t received() const { return received_; }
private:
  void record(const std::string& status, double rtt = 0.0) {
    if (next_seq_ <= warmup_) return;
    output_ << csv_escape(run_id_) << ',' << next_seq_ << ',' << payload_
            << ',' << status << ',';
    if (status == "ok") output_ << std::fixed << std::setprecision(3)
                                << rtt << ',' << rtt / 2.0;
    else output_ << ',';
    output_ << '\n';
    output_.flush();
  }
  void receive(const Packet& msg) {
    if (!in_flight_ || msg.data.size() != payload_ || msg.data.size() < 8 ||
        get_sequence(msg.data) != next_seq_) return;
    const auto now = Clock::now();
    in_flight_ = false;
    const double us = std::chrono::duration<double, std::micro>(now - sent_).count();
    record("ok", us);
    if (next_seq_ > warmup_) ++received_;
    ++next_seq_;
  }
  void advance() {
    if (finished_) return;
    if (in_flight_) {
      if (Clock::now() - sent_ < timeout_) return;
      in_flight_ = false;
      record("timeout");
      if (next_seq_ > warmup_) ++lost_;
      ++next_seq_;
    }
    if (next_seq_ > warmup_ + target_) {
      finished_ = true;
      output_.flush();
      return;
    }
    Packet request;
    put_sequence(data_, next_seq_);
    request.data = data_;
    sent_ = Clock::now();
    in_flight_ = true;
    publisher_->publish(request);
  }
  size_t payload_;
  uint64_t target_, warmup_, next_seq_{1}, received_{0}, lost_{0};
  std::chrono::milliseconds timeout_;
  std::string run_id_;
  std::vector<uint8_t> data_;
  std::ofstream output_;
  Clock::time_point sent_;
  bool in_flight_{false}, finished_{false};
  rclcpp::Publisher<Packet>::SharedPtr publisher_;
  rclcpp::Subscription<Packet>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr tick_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int ret = 0;
  try {
    // ROS parameters let callers set all workload dimensions without recompilation.
    auto config = std::make_shared<rclcpp::Node>("robotcore_dds_config");
    const auto role = config->declare_parameter<std::string>("role", "responder");
    const auto payload_i = config->declare_parameter<int64_t>("payload_bytes", 64);
    const auto samples_i = config->declare_parameter<int64_t>("samples", 1000);
    const auto warmup_i = config->declare_parameter<int64_t>("warmup", 100);
    const auto timeout_i = config->declare_parameter<int64_t>("timeout_ms", 1000);
    const auto depth_i = config->declare_parameter<int64_t>("qos_depth", 10);
    const auto reliability = config->declare_parameter<std::string>("reliability", "reliable");
    const auto csv = config->declare_parameter<std::string>("csv", "baseline.csv");
    const auto run_id = config->declare_parameter<std::string>("run_id", "manual");
    if (payload_i < 8 || payload_i > 16 * 1024 * 1024 || samples_i < 1 ||
        warmup_i < 0 || timeout_i < 1 || depth_i < 1 || depth_i > 10000 ||
        (reliability != "reliable" && reliability != "best_effort")) {
      throw std::invalid_argument("invalid payload/samples/warmup/timeout/qos/reliability");
    }
    const rclcpp::QoS qos = reliability == "reliable"
      ? rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(depth_i))).reliable()
      : rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(depth_i))).best_effort();
    if (role == "responder") {
      rclcpp::spin(std::make_shared<Responder>(static_cast<size_t>(payload_i), qos));
    } else if (role == "initiator") {
      auto node = std::make_shared<Initiator>(static_cast<size_t>(payload_i),
        static_cast<uint64_t>(samples_i), static_cast<uint64_t>(warmup_i),
        std::chrono::milliseconds(timeout_i), csv, run_id, qos);
      rclcpp::executors::SingleThreadedExecutor executor;
      executor.add_node(node);
      while (rclcpp::ok() && !node->done()) executor.spin_some(10ms);
      std::cout << "received=" << node->received() << " lost=" << node->lost() << '\n';
      if (node->lost() > 0) ret = 2;
    } else {
      throw std::invalid_argument("role must be initiator or responder");
    }
  } catch (const std::exception& ex) {
    std::cerr << "error: " << ex.what() << '\n';
    ret = 1;
  }
  rclcpp::shutdown();
  return ret;
}

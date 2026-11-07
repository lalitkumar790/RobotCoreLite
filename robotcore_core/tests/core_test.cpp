#include "robotcore/core.hpp"
#include <cassert>
#include <utility>
using namespace robotcore;

class Loopback final : public Transport {
public:
  explicit Loopback(Configuration c) : config_(std::move(c)) {}
  Status start(ReceiveCallback cb) override {
    if (started_) return Status::failure(Error::already_started, "already started");
    if (!cb) return Status::failure(Error::invalid_argument, "callback required");
    const auto status = validate(config_);
    if (!status) return status;
    on_receive_ = std::move(cb);
    started_ = true;
    return Status::success();
  }
  Status publish(const Message& m) override {
    if (!started_) return Status::failure(Error::not_started, "not started");
    const auto status = validate_message(m, config_);
    if (!status) return status;
    on_receive_(m); // deliberate copy: transport may not borrow caller's buffer.
    return Status::success();
  }
  Status stop() override {
    started_ = false;
    on_receive_ = {};
    return Status::success();
  }
  TransportKind kind() const noexcept override { return TransportKind::dds; }
private:
  Configuration config_;
  ReceiveCallback on_receive_;
  bool started_{false};
};

int main() {
  Configuration c;
  assert(validate(c));
  assert(resolve_transport(c).second == TransportKind::dds);
  c.transport = TransportKind::ipc;
  assert(resolve_transport(c).first.error == Error::unsupported);
  c.allow_dds_fallback = true;
  assert(resolve_transport(c).first);
  assert(resolve_transport(c).second == TransportKind::dds);
  c.transport = TransportKind::shared_memory;
  assert(resolve_transport(c).second == TransportKind::dds);
  c.transport = TransportKind::persistent_shared_memory;
  assert(resolve_transport(c).second == TransportKind::dds);
  c.queue_depth = 0;
  assert(validate(c).error == Error::invalid_argument);
  c.queue_depth = 10;
  c.max_payload_bytes = 4;
  Message received;
  bool called = false;
  Loopback transport(c);
  assert(transport.publish({1, {1}}).error == Error::not_started);
  assert(transport.start([&](Message m) { received = std::move(m); called = true; }));
  assert(transport.start([](Message) {}).error == Error::already_started);
  Message sent{42, {1, 2, 3}};
  assert(transport.publish(sent));
  sent.payload[0] = 99;
  assert(called && received.sequence == 42 && received.payload[0] == 1);
  assert(transport.publish({43, {1, 2, 3, 4, 5}}).error == Error::too_large);
  assert(transport.stop());
  assert(transport.publish(sent).error == Error::not_started);
  assert(transport.start({}).error == Error::invalid_argument);
  assert(transport.start([](Message) {}));
  assert(transport.stop());
}

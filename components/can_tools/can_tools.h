#pragma once

#include "esphome/components/canbus/canbus.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/optional.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#include <functional>
#include <utility>
#include <vector>

namespace esphome::can_tools {

using Payload = std::vector<uint8_t>;

/// Diagnostic request (OBD-II, UDS, manufacturer services) sent every interval as an ISO-TP single frame,
/// the response is reassembled from ISO-TP frames and handed on as its payload (response SID first).
class Request {
  friend class CanTools;

 public:
  Request(uint32_t can_id, uint32_t response_can_id, Payload data, uint32_t interval, uint32_t timeout)
      : can_id_(can_id),
        response_can_id_(response_can_id),
        data_(std::move(data)),
        interval_(interval),
        timeout_(timeout) {}

  template<typename F> void add_on_response_callback(F &&callback) {
    this->callbacks_.add(std::forward<F>(callback));
  }
#ifdef USE_SENSOR
  void add_sensor(sensor::Sensor *s, std::function<optional<float>(const Payload &)> &&decode) {
    this->add_on_response_callback([s, decode](const Payload &data) {
      if (auto v = decode(data))
        s->publish_state(*v);
    });
  }
#endif
#ifdef USE_BINARY_SENSOR
  void add_binary_sensor(binary_sensor::BinarySensor *s, std::function<optional<bool>(const Payload &)> &&decode) {
    this->add_on_response_callback([s, decode](const Payload &data) {
      if (auto v = decode(data))
        s->publish_state(*v);
    });
  }
#endif

 protected:
  uint32_t can_id_;
  uint32_t response_can_id_;
  Payload data_;
  uint32_t interval_;
  uint32_t timeout_;
  uint32_t last_sent_{0};
  bool sent_once_{false};
  LazyCallbackManager<void(const Payload &)> callbacks_;
};

/// CAN bus tools on top of a canbus component. Now: diagnostic requests, one at a time.
/// update_interval is the scheduling tick; component.suspend stops sending (e.g. while the car sleeps).
class CanTools : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_canbus(canbus::Canbus *canbus) { this->canbus_ = canbus; }
  void add_request(Request *request) { this->requests_.push_back(request); }

 protected:
  void on_frame_(uint32_t can_id, bool extended_id, bool rtr, const Payload &frame);
  void send_(uint32_t can_id, const uint8_t *data, size_t len);
  void finish_(const char *reason);

  canbus::Canbus *canbus_{nullptr};
  std::vector<Request *> requests_;
  size_t next_{0};
  Request *active_{nullptr};
  uint32_t active_since_{0};
  // ISO-TP reassembly of the active request's response
  Payload rx_;
  size_t rx_len_{0};
  uint8_t rx_seq_{0};
};

class ResponseTrigger : public Trigger<Payload> {
 public:
  explicit ResponseTrigger(Request *request) {
    request->add_on_response_callback([this](const Payload &data) { this->trigger(data); });
  }
};

}  // namespace esphome::can_tools

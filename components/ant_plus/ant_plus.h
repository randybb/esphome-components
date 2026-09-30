#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <vector>

namespace esphome::ant_plus {

/// ANT+ receiver on the chip's own radio, using RadiANT (clean-room ANT+ compatible
/// link layer) through its antr_* API. Messages arrive on RadiANT's event thread, so
/// they only update atomics and loop() publishes.
class AntPlus : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_network_key(const std::vector<uint8_t> &key) { std::copy_n(key.begin(), 8, this->network_key_.begin()); }
  void set_device_number(uint32_t device_number) { this->device_number_ = device_number; }
  void set_heart_rate_sensor(sensor::Sensor *sensor) { this->heart_rate_sensor_ = sensor; }

  // RadiANT event thread
  void on_message(uint8_t id, const uint8_t *data, uint8_t len);

 protected:
  std::array<uint8_t, 8> network_key_{};
  uint32_t device_number_{0};
  sensor::Sensor *heart_rate_sensor_{nullptr};

  std::atomic<uint8_t> heart_rate_{0};
  std::atomic<uint32_t> last_page_ms_{0};
  std::atomic<bool> have_data_{false};
  std::atomic<bool> reopen_{false};
  std::atomic<uint8_t> last_event_{0};
  uint32_t paired_id_{0};
  bool published_nan_{true};
};

}  // namespace esphome::ant_plus

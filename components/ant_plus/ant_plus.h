#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include <algorithm>
#include <array>
#include <atomic>
#include <vector>

namespace esphome::ant_plus {

enum class DeviceType : uint8_t { HEART_RATE, FITNESS_EQUIPMENT, TEMPERATURE, BIKE_RADAR };

/// One ANT+ sensor on its own channel. Pages are decoded in the main loop.
class AntPlusDevice {
 public:
  AntPlusDevice(DeviceType type, uint32_t ant_id) : type_(type), ant_id_(ant_id) {}

  DeviceType get_type() const { return this->type_; }
  uint32_t get_ant_id() const { return this->ant_id_; }
  uint8_t ant_device_type() const;
  uint16_t channel_period() const;

  void on_page(const uint8_t *page, uint32_t now);
  void check_stale(uint32_t now);
  bool is_connected() const { return this->connected_state_; }

#ifdef USE_SENSOR
  void set_heart_rate_sensor(sensor::Sensor *s) { this->heart_rate_ = s; }
  void set_battery_level_sensor(sensor::Sensor *s) { this->battery_level_ = s; }
  void set_battery_voltage_sensor(sensor::Sensor *s) { this->battery_voltage_ = s; }
  void set_power_sensor(sensor::Sensor *s) { this->power_ = s; }
  void set_cadence_sensor(sensor::Sensor *s) { this->cadence_ = s; }
  void set_speed_sensor(sensor::Sensor *s) { this->speed_ = s; }
  void set_distance_sensor(sensor::Sensor *s) { this->distance_ = s; }
  void set_temperature_sensor(sensor::Sensor *s) { this->temperature_ = s; }
  void set_temperature_min_sensor(sensor::Sensor *s) { this->temperature_min_ = s; }
  void set_temperature_max_sensor(sensor::Sensor *s) { this->temperature_max_ = s; }
#endif
#ifdef USE_BINARY_SENSOR
  void set_connected_binary_sensor(binary_sensor::BinarySensor *s) { this->connected_ = s; }
  void set_in_use_binary_sensor(binary_sensor::BinarySensor *s) { this->in_use_ = s; }
  void set_battery_low_binary_sensor(binary_sensor::BinarySensor *s) { this->battery_low_ = s; }
#endif

 protected:
  void decode_heart_rate_(const uint8_t *page);
  void decode_fitness_equipment_(const uint8_t *page);
  void decode_temperature_(const uint8_t *page);
  void decode_battery_(uint8_t fractional, uint8_t coarse);
  void set_connected_(bool connected);

  DeviceType type_;
  uint32_t ant_id_;
  uint32_t last_page_ms_{0};
  bool connected_state_{false};
  // FE rollover counters (distance in m, one byte on air)
  int last_distance_{-1};
  uint32_t total_distance_{0};

#ifdef USE_SENSOR
  sensor::Sensor *heart_rate_{nullptr};
  sensor::Sensor *battery_level_{nullptr};
  sensor::Sensor *battery_voltage_{nullptr};
  sensor::Sensor *power_{nullptr};
  sensor::Sensor *cadence_{nullptr};
  sensor::Sensor *speed_{nullptr};
  sensor::Sensor *distance_{nullptr};
  sensor::Sensor *temperature_{nullptr};
  sensor::Sensor *temperature_min_{nullptr};
  sensor::Sensor *temperature_max_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *connected_{nullptr};
  binary_sensor::BinarySensor *in_use_{nullptr};
  binary_sensor::BinarySensor *battery_low_{nullptr};
#endif
};

/// ANT+ receiver on the chip's own radio, using RadiANT (clean-room ANT+ compatible
/// link layer) through its antr_* API. One slave channel per configured device.
class AntPlus : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_network_key(const std::vector<uint8_t> &key) { std::copy_n(key.begin(), 8, this->network_key_.begin()); }
  void add_device(AntPlusDevice *device) { this->devices_.push_back(device); }
#ifdef USE_BINARY_SENSOR
  void set_connected_binary_sensor(binary_sensor::BinarySensor *s) { this->connected_ = s; }
  void set_unknown_device_binary_sensor(binary_sensor::BinarySensor *s) { this->unknown_device_ = s; }
#endif

  // RadiANT event thread: queue it, loop() decodes
  void on_message(uint8_t id, const uint8_t *data, uint8_t len);

 protected:
  struct Message {
    uint8_t id;
    uint8_t channel;
    uint8_t data[8];
  };
  static constexpr uint8_t QUEUE_SIZE = 32;

  bool open_channel_(uint8_t channel);
  bool setup_discovery_channel_(uint8_t channel);
  void on_discovery_page_(uint32_t now);

  // extra wildcard channel with the known devices excluded, when unknown_device is set
  bool discovery_{false};
  bool unknown_state_{false};
  uint32_t last_unknown_ms_{0};
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *connected_{nullptr};
  binary_sensor::BinarySensor *unknown_device_{nullptr};
#endif

  std::array<uint8_t, 8> network_key_{};
  std::vector<AntPlusDevice *> devices_;

  // single producer (RadiANT thread), single consumer (loop)
  std::array<Message, QUEUE_SIZE> queue_{};
  std::atomic<uint8_t> head_{0};
  std::atomic<uint8_t> tail_{0};
  std::atomic<uint32_t> dropped_{0};
};

}  // namespace esphome::ant_plus

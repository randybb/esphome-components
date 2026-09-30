#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"

namespace esphome::ant_plus {

/// Reads the ANT+ bridge (nrf-ant-bridge firmware on an nRF52) over UART. The bridge prints
/// "HRM <ANT ID> <heart rate>" for every page it receives from a heart rate monitor;
/// other lines are its Zephyr log, which is forwarded to the ESPHome log.
class AntPlus : public Component, public uart::UARTDevice {
 public:
  void loop() override;
  void dump_config() override;

  void set_device_number(uint32_t device_number) { this->device_number_ = device_number; }
  void set_heart_rate_sensor(sensor::Sensor *sensor) { this->heart_rate_sensor_ = sensor; }

 protected:
  void handle_line_();

  sensor::Sensor *heart_rate_sensor_{nullptr};
  uint32_t device_number_{0};
  char line_[128];
  size_t line_len_{0};
  uint32_t last_page_ms_{0};
  bool published_nan_{true};
};

}  // namespace esphome::ant_plus

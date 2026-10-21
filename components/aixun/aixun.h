#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/core/line_reader.h"

#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#include <cmath>

namespace esphome::aixun {

/// Aixun device on its UART or USB port (FT232 emulation), text console at 115200 Bd.
///
/// Any model: JC_version gives the model and firmware version. P2408: JCXX_open starts a ~100 Hz status stream,
/// the latest line is kept and published every update_interval.
class Aixun : public PollingComponent, public uart::UARTDevice {
 public:
  void loop() override;
  void update() override;
  void dump_config() override;

#ifdef USE_SENSOR
  /// out: 0 = PD+QC, 1 = Boot, 2 = DC; kind: 0 = voltage, 1 = current, 2 = power
  void set_sensor(int out, int kind, sensor::Sensor *s) { this->sensors_[out][kind] = s; }
#endif
#ifdef USE_NUMBER
  void set_dc_limit_number(number::Number *n) { this->dc_limit_number_ = n; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_device_model_text_sensor(text_sensor::TextSensor *s) { this->device_model_ = s; }
  void set_firmware_version_text_sensor(text_sensor::TextSensor *s) { this->firmware_version_ = s; }
#endif
  void set_p2408(bool p2408) { this->p2408_ = p2408; }

  void set_dc_output(bool on);
  /// The supply takes both limits at once, the Boot one is sent back as last read.
  bool set_dc_current_limit(float amps);

 protected:
  void send_(const char *cmd);
  void handle_line_(char *line);

#ifdef USE_SENSOR
  sensor::Sensor *sensors_[3][3]{};
#endif
#ifdef USE_NUMBER
  number::Number *dc_limit_number_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *device_model_{nullptr};
  text_sensor::TextSensor *firmware_version_{nullptr};
#endif
  bool p2408_{false};
  bool have_version_{false};
  uint32_t last_version_request_{0};
  uint32_t last_byte_{0};
  float vi_[3][2]{};
  bool fresh_{false};
  float batt_limit_{NAN};
  LineReader<160> line_reader_;
  uint32_t last_status_{0};
  uint32_t last_open_{0};
  bool opened_{false};
};

#ifdef USE_SWITCH
class DCOutputSwitch : public switch_::Switch, public Parented<Aixun> {
 public:
  bool assumed_state() override { return true; }

 protected:
  void write_state(bool state) override {
    this->parent_->set_dc_output(state);
    this->publish_state(state);
  }
};
#endif

#ifdef USE_NUMBER
class DCCurrentLimitNumber : public number::Number, public Parented<Aixun> {
 protected:
  // The state is published when the supply reports the new limit back.
  void control(float value) override { this->parent_->set_dc_current_limit(value); }
};
#endif

}  // namespace esphome::aixun

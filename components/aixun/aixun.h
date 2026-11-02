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
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
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
/// the latest line is kept and published every update_interval. H310D: ESPHome on the station's own ESP32-C3,
/// a binary protocol to the GD32 main MCU instead of the text console (see the h310d_* members).
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
  void set_serial_number_text_sensor(text_sensor::TextSensor *s) { this->serial_number_ = s; }
#endif
#ifdef USE_SENSOR
  /// H310D status: 0 = current temperature, 1 = current fan speed, 2 = heating power
  void set_h310d_sensor(int idx, sensor::Sensor *s) { this->h310d_sensors_[idx] = s; }
#endif
#ifdef USE_NUMBER
  void set_target_temperature_number(number::Number *n) { this->temp_number_ = n; }
  void set_fan_speed_number(number::Number *n) { this->fan_number_ = n; }
#endif
#ifdef USE_TEXT_SENSOR
  void set_heater_status_text_sensor(text_sensor::TextSensor *s) { this->heater_status_ = s; }
#endif
#ifdef USE_BINARY_SENSOR
  void set_sta_binary_sensor(binary_sensor::BinarySensor *s) { this->sta_bsensor_ = s; }
  void set_ap_binary_sensor(binary_sensor::BinarySensor *s) { this->ap_bsensor_ = s; }
#endif
  void set_p2408(bool p2408) { this->p2408_ = p2408; }
  void set_h310d(bool h310d) { this->h310d_ = h310d; }

  /// H310D: the remote's setter TLVs (cmd 0x04). No readback yet, so the entities are optimistic.
  void h310d_set_temperature(int celsius);
  void h310d_set_fan(int speed);
  /// H310D: recall preset M1..M3 (short press of the remote's preset key, never saves).
  void h310d_recall_preset(int m);

  void set_dc_output(bool on);
  /// The supply takes both limits at once, the Boot one is sent back as last read.
  bool set_dc_current_limit(float amps);

 protected:
  void send_(const char *cmd);
  void handle_line_(char *line);

  // --- H310D: ESPHome runs on the station's own ESP32-C3, binary link to the GD32 MCU (UART0, 115200).
  // Frame: [cmd][flag][len16 LE = total][00][00][00][payload][crc8 poly 0x07]. flag 0x00 = to MCU, 0xAA = reply.
  // loop() only assembles frames and stores state + sends heartbeat/info on timers; update() publishes on change.
  void h310d_loop_(uint32_t now);
  void h310d_update_();
  void h310d_dispatch_(const uint8_t *f, uint16_t len);
  void h310d_send_(uint8_t cmd, const uint8_t *payload, uint8_t plen);
  void h310d_send_connect_();
  static uint8_t crc8_(const uint8_t *d, uint16_t n);

#ifdef USE_SENSOR
  sensor::Sensor *sensors_[3][3]{};
#endif
#ifdef USE_NUMBER
  number::Number *dc_limit_number_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *device_model_{nullptr};
  text_sensor::TextSensor *firmware_version_{nullptr};
  text_sensor::TextSensor *serial_number_{nullptr};
#endif
#ifdef USE_BINARY_SENSOR
  binary_sensor::BinarySensor *sta_bsensor_{nullptr};
  binary_sensor::BinarySensor *ap_bsensor_{nullptr};
#endif
#ifdef USE_SENSOR
  sensor::Sensor *h310d_sensors_[3]{};
#endif
#ifdef USE_NUMBER
  number::Number *temp_number_{nullptr};
  number::Number *fan_number_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *heater_status_{nullptr};
#endif
  // H310D binary-protocol state
  bool h310d_{false};
  // Status pushed to a paired remote (cmd 0x02, TLV 0x39): busy, heating power %,
  // real temp, real fan, set temp, set fan.
  bool h310d_st_busy_{false};
  uint8_t h310d_st_power_{0};
  int16_t h310d_st_real_temp_{0}, h310d_st_real_fan_{0}, h310d_st_set_temp_{0}, h310d_st_set_fan_{0};
  bool h310d_st_fresh_{false};
  bool h310d_connected_{false};
  uint8_t h310d_buf_[256];
  uint16_t h310d_len_{0};
  uint32_t h310d_last_hb_{0};
  uint32_t h310d_last_info_{0};
  std::string h310d_model_, h310d_serial_, h310d_fw_;
  bool h310d_info_dirty_{false};
  char h310d_sta_{0}, h310d_ap_{0};  // UI-state TLV 0x50 / 0x53 values, 0 = not reported yet
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

#ifdef USE_NUMBER
class H310DTemperatureNumber : public number::Number, public Parented<Aixun> {
 protected:
  void control(float value) override {
    this->parent_->h310d_set_temperature(static_cast<int>(value));
    this->publish_state(value);
  }
};

class H310DFanNumber : public number::Number, public Parented<Aixun> {
 protected:
  void control(float value) override {
    this->parent_->h310d_set_fan(static_cast<int>(value));
    this->publish_state(value);
  }
};
#endif

#ifdef USE_BUTTON
class H310DPresetButton : public button::Button, public Parented<Aixun> {
 public:
  void set_preset(int m) { this->preset_ = m; }

 protected:
  void press_action() override { this->parent_->h310d_recall_preset(this->preset_); }
  int preset_{1};
};
#endif

}  // namespace esphome::aixun

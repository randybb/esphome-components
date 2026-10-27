#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/output/float_output.h"
#include <vector>

namespace esphome::p9813 {

/// P9813 RGB LED drivers in a chain, each of their 3 channels as a separate output.
class P9813 final : public Component {
 public:
  class Channel;

  void set_data_pin(GPIOPin *data_pin) { data_pin_ = data_pin; }
  void set_clock_pin(GPIOPin *clock_pin) { clock_pin_ = clock_pin; }
  void set_num_chips(uint8_t num_chips) { num_chips_ = num_chips; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  /// Send new values if they were updated.
  void loop() override;

  class Channel final : public output::FloatOutput {
   public:
    void set_parent(P9813 *parent) { parent_ = parent; }
    void set_channel(uint8_t channel) { channel_ = channel; }

   protected:
    void write_state(float state) override { this->parent_->set_channel_value_(this->channel_, state * 0xFF); }

    P9813 *parent_;
    uint8_t channel_;
  };

 protected:
  void set_channel_value_(uint8_t channel, uint8_t value);
  void write_byte_(uint8_t data);

  GPIOPin *data_pin_;
  GPIOPin *clock_pin_;
  uint8_t num_chips_;
  std::vector<uint8_t> pwm_amounts_;  // R, G, B of each chip
  bool update_{true};
};

}  // namespace esphome::p9813

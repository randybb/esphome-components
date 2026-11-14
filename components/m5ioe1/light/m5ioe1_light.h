#pragma once

#include "esphome/components/light/addressable_light.h"
#include "esphome/components/light/light_output.h"

#include "../m5ioe1.h"

namespace esphome::m5ioe1 {

class M5IOE1AddressableLight : public light::AddressableLight,
                               public Parented<M5IOE1Component>
{
  public:
    void setup() override;
    void write_state(light::LightState *state) override;

    float get_setup_priority() const override { return setup_priority::HARDWARE; }
    void set_num_led(int32_t num) { this->num_leds_ = num; }

    int32_t size() const override { return this->num_leds_; }

    light::LightTraits get_traits() override {
      auto traits = light::LightTraits();
      traits.set_supported_color_modes({light::ColorMode::RGB});
      return traits;
    };

    void clear_effect_data() override { this->internal_accessor_.clear_effect_data(this->num_leds_); }

  protected:
    int32_t num_leds_;

    // The on device driver only support RGB565 (16 bits)
    // but the buffer is still RGB888 (24 bits) format for compatibility
    // data will be truncated when writing registers.
    light::CorrectedBufferColorAccessor internal_accessor_{nullptr, nullptr, {0, 1, 2}, &this->correction_};
};                              


} // namespace esphome::m5ioe1
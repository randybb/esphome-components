#include "m5ioe1_light.h"
#include "esphome/core/log.h"
#include <cstring>

namespace esphome::m5ioe1 {

static const char *TAG = "m5ioe1.light";

void M5IOE1AddressableLight::setup() {
  RAMAllocator<uint8_t> allocator;
  if (!this->internal_accessor_.allocate(allocator, this->num_leds_)) {
    ESP_LOGE(TAG, "Failed to allocate buffer.");
    this->mark_failed();
    return;
  }
  this->accessor_ = &this->internal_accessor_;

  // Enable LED, refresh RAM datazone
  this->parent_->setup_neopixel(this->num_leds_);

}


void M5IOE1AddressableLight::write_state(light::LightState *state) {
  uint8_t hwbuf[this->num_leds_ * 2];
  
  const uint8_t *src = this->internal_accessor_.color_data;
  uint8_t *dst = hwbuf;

  for (int i = 0; i < num_leds_; i++) {
    uint8_t r = *src++;
    uint8_t g = *src++;
    uint8_t b = *src++;

    uint16_t rgb565 =
        ((r & 0xF8) << 8) |
        ((g & 0xFC) << 3) |
        (b >> 3);

    *dst++ = rgb565 & 0xFF;
    *dst++ = (rgb565 >> 8) & 0xFF;
  }

  // Write RAM data
  this->parent_->set_led_color(hwbuf, sizeof(hwbuf));
}

}  // namespace esphome::m5ioe1
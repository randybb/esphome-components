#include "p9813.h"
#include "esphome/core/log.h"

namespace esphome::p9813 {

ESPHOME_LOG_TAG(TAG, "p9813");

void P9813::setup() {
  this->data_pin_->setup();
  this->data_pin_->digital_write(false);
  this->clock_pin_->setup();
  this->clock_pin_->digital_write(false);
  this->pwm_amounts_.resize(this->num_chips_ * 3, 0);
}

void P9813::dump_config() {
  ESP_LOGCONFIG(TAG,
                "P9813:\n"
                "  Number of chips: %u",
                this->num_chips_);
  LOG_PIN("  Data Pin: ", this->data_pin_);
  LOG_PIN("  Clock Pin: ", this->clock_pin_);
}

void P9813::set_channel_value_(uint8_t channel, uint8_t value) {
  if (channel >= this->pwm_amounts_.size()) {
    ESP_LOGW(TAG, "Channel %u out of range, %u chips have %u channels", channel, this->num_chips_,
             this->pwm_amounts_.size());
    return;
  }
  if (this->pwm_amounts_[channel] != value)
    this->update_ = true;
  this->pwm_amounts_[channel] = value;
}

void P9813::write_byte_(uint8_t data) {
  // Data is latched on the rising edge of the clock
  for (uint8_t mask = 0x80; mask; mask >>= 1) {
    this->data_pin_->digital_write(data & mask);
    this->clock_pin_->digital_write(true);
    this->clock_pin_->digital_write(false);
  }
}

void P9813::loop() {
  if (!this->update_)
    return;

  // 32 zero bits start the frame, then 32 bits per chip, the first chip first
  for (uint8_t i = 0; i < 4; i++)
    this->write_byte_(0);
  for (size_t i = 0; i < this->pwm_amounts_.size(); i += 3) {
    uint8_t r = this->pwm_amounts_[i], g = this->pwm_amounts_[i + 1], b = this->pwm_amounts_[i + 2];
    // Flag byte: 0b11, then the inverted two top bits of B, G and R as a checksum
    this->write_byte_(0xC0 | ((~b & 0xC0) >> 2) | ((~g & 0xC0) >> 4) | ((~r & 0xC0) >> 6));
    this->write_byte_(b);
    this->write_byte_(g);
    this->write_byte_(r);
  }
  for (uint8_t i = 0; i < 4; i++)
    this->write_byte_(0);

  this->update_ = false;
}

}  // namespace esphome::p9813

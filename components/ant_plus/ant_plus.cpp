#include "ant_plus.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome::ant_plus {

static const char *const TAG = "ant_plus";

// No page for this long means the monitor is gone (off, out of range, no skin contact)
static constexpr uint32_t STALE_MS = 5000;

void AntPlus::loop() {
  uint8_t c;
  while (this->available() && this->read_byte(&c)) {
    if (c == '\n') {
      this->line_[this->line_len_] = '\0';
      this->handle_line_();
      this->line_len_ = 0;
    } else if (c != '\r' && this->line_len_ < sizeof(this->line_) - 1) {
      this->line_[this->line_len_++] = c;
    }
  }

  if (!this->published_nan_ && millis() - this->last_page_ms_ > STALE_MS) {
    if (this->heart_rate_sensor_ != nullptr)
      this->heart_rate_sensor_->publish_state(NAN);
    this->published_nan_ = true;
  }
}

void AntPlus::handle_line_() {
  uint32_t id, heart_rate;
  if (std::sscanf(this->line_, "HRM %" SCNu32 " %" SCNu32, &id, &heart_rate) != 2) {
    // the bridge's Zephyr log ("<err> module: message"), boot banner etc.
    if (this->line_len_ == 0)
      return;
    const char *message = this->line_ + 6;
    if (this->line_len_ > 6 && std::strncmp(this->line_, "<err> ", 6) == 0) {
      ESP_LOGE(TAG, "Bridge: %s", message);
    } else if (this->line_len_ > 6 && std::strncmp(this->line_, "<wrn> ", 6) == 0) {
      ESP_LOGW(TAG, "Bridge: %s", message);
    } else if (this->line_len_ > 6 && std::strncmp(this->line_, "<inf> ", 6) == 0) {
      ESP_LOGI(TAG, "Bridge: %s", message);
    } else {
      ESP_LOGD(TAG, "Bridge: %s", this->line_);
    }
    return;
  }
  if (this->device_number_ != 0 && id != this->device_number_)
    return;
  this->last_page_ms_ = millis();
  this->published_nan_ = false;
  if (this->heart_rate_sensor_ != nullptr)
    this->heart_rate_sensor_->publish_state(heart_rate);
}

void AntPlus::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ANT+:\n"
                "  Heart rate monitor ANT ID: %" PRIu32 "%s",
                this->device_number_, this->device_number_ == 0 ? " (any)" : "");
  LOG_SENSOR("  ", "Heart Rate", this->heart_rate_sensor_);
}

}  // namespace esphome::ant_plus

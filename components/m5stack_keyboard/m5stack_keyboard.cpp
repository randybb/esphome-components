#include "m5stack_keyboard.h"
#include "esphome/core/log.h"

namespace esphome::m5stack_keyboard {

static const char *const TAG = "m5stack_keyboard";

void M5StackKeyboard::update() {
  uint8_t key;
  if (this->read(&key, 1) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }
  this->status_clear_warning();
  if (key == 0)
    return;
  ESP_LOGV(TAG, "Key 0x%02X", key);
  this->send_key_(key);
}

void M5StackKeyboard::dump_config() {
  ESP_LOGCONFIG(TAG, "M5Stack keyboard:");
  LOG_I2C_DEVICE(this);
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace esphome::m5stack_keyboard

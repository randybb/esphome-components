#pragma once

#include "esphome/components/i2c/i2c.h"
#include "esphome/components/key_provider/key_provider.h"
#include "esphome/core/component.h"

namespace esphome::m5stack_keyboard {

/// M5Stack Faces QWERTY / CardKB: one byte per read, the next key in the queue or 0
class M5StackKeyboard : public PollingComponent, public i2c::I2CDevice, public key_provider::KeyProvider {
 public:
  void update() override;
  void dump_config() override;
};

}  // namespace esphome::m5stack_keyboard

#pragma once

#include "esphome/components/display/display_buffer.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome::m5stack_st7789v2 {

static constexpr int WIDTH = 135;
static constexpr int HEIGHT = 240;

/// M5Stack Unit LCD (U120): an ESP32-PICO in the unit takes LovyanGFX's command set over I2C
/// and drives the ST7789V2 itself. The frame is kept here in RGB565 and only the rows that differ
/// from what the unit shows go out, a few per loop() so a full frame (about 1.7 s at 400 kHz)
/// does not block the main loop.
class M5StackST7789V2 : public display::DisplayBuffer, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  /// Backlight, 0.0 to 1.0.
  void set_brightness(float brightness);

  display::DisplayType get_display_type() override { return display::DisplayType::DISPLAY_TYPE_COLOR; }
  /// LVGL flushes: RGB565 blocks are copied into the frame, the rows go out from loop().
  void draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr, display::ColorOrder order,
                      display::ColorBitness bitness, bool big_endian, int x_offset, int y_offset, int x_pad) override;

 protected:
  void draw_absolute_pixel_internal(int x, int y, Color color) override;
  int get_width_internal() override { return WIDTH; }
  int get_height_internal() override { return HEIGHT; }

  bool connect_();
  bool wait_buffer_(uint8_t needed);

  bool connected_{false};
  uint8_t brightness_{255};
  uint8_t free_{0};
  // Next row to compare with what the unit shows, HEIGHT when all are sent.
  int row_{HEIGHT};
  uint32_t sent_hash_[HEIGHT]{};
};

}  // namespace esphome::m5stack_st7789v2

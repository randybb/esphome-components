#include "m5stack_st7789v2.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::m5stack_st7789v2 {

static const char *const TAG = "m5stack_st7789v2";

// From LovyanGFX Panel_M5UnitLCD, coordinates are one byte each (the panel is under 256 px).
static constexpr uint8_t CMD_READ_ID = 0x04;        // answers 0x77 0x89 <major> <minor>
static constexpr uint8_t CMD_READ_BUFCOUNT = 0x09;  // answers the free command slots, up to 255
static constexpr uint8_t CMD_BRIGHTNESS = 0x22;     // [1] backlight 0-255
static constexpr uint8_t CMD_CASET = 0x2A;          // [1] xs [2] xe
static constexpr uint8_t CMD_RASET = 0x2B;          // [1] ys [2] ye
static constexpr uint8_t CMD_ROTATE = 0x36;         // [1] 0-3, the unit may start rotated
static constexpr uint8_t CMD_WRITE_RAW_16 = 0x42;   // RGB565 pixels, high byte first, into the window
static constexpr uint8_t CMD_WRITE_RLE_16 = 0x4A;   // RGB565 high byte first: [n, pixel] runs, [0, n, n pixels] literals

// Time spent sending rows per loop().
static constexpr uint32_t LOOP_BUDGET_MS = 20;

void M5StackST7789V2::setup() {
  this->init_internal_(WIDTH * HEIGHT * 2);
  if (this->buffer_ == nullptr) {
    this->mark_failed(LOG_STR("frame buffer allocation failed"));
    return;
  }
  this->connect_();
}

bool M5StackST7789V2::connect_() {
  uint8_t cmd = CMD_READ_ID;
  uint8_t id[4];
  if (this->write(&cmd, 1) != i2c::ERROR_OK || this->read(id, sizeof(id)) != i2c::ERROR_OK || id[0] != 0x77 ||
      id[1] != 0x89) {
    ESP_LOGW(TAG, "Unit LCD not answering");
    return false;
  }
  ESP_LOGD(TAG, "Unit LCD firmware %u.%u", id[2], id[3]);
  // The unit answers READ_ID about 250 ms before it draws: pixels sent earlier are dropped.
  delay(300);
  // Our frame is native portrait, the ESPHome rotation is applied when drawing into it
  const uint8_t rotate[] = {CMD_ROTATE, 0};
  this->write(rotate, sizeof(rotate));
  this->connected_ = true;
  this->free_ = 0;
  this->set_brightness(this->brightness_ / 255.0f);
  // Its frame is unknown after a (re)start, send ours whole.
  memset(this->sent_hash_, 0, sizeof(this->sent_hash_));
  this->row_ = 0;
  this->enable_loop();
  return true;
}

void M5StackST7789V2::update() {
  if (!this->connected_ && !this->connect_())
    return;
  this->do_update_();
  this->row_ = 0;
  this->enable_loop();
}

void M5StackST7789V2::set_brightness(float brightness) {
  this->brightness_ = lroundf(brightness * 255);
  if (!this->connected_)
    return;
  const uint8_t data[] = {CMD_BRIGHTNESS, this->brightness_};
  this->write(data, sizeof(data));
}

// ponytail: the unit's command buffer counts commands, a pixel stream is charged like
// LovyanGFX does (a slot per 4 pixels); measure on the unit if rows get lost.
bool M5StackST7789V2::wait_buffer_(uint8_t needed) {
  if (this->free_ < needed) {
    uint8_t cmd = CMD_READ_BUFCOUNT;
    if (this->write(&cmd, 1) != i2c::ERROR_OK || this->read(&this->free_, 1) != i2c::ERROR_OK) {
      this->free_ = 0;
      return false;
    }
    if (this->free_ < needed)
      return false;
  }
  this->free_ -= needed;
  return true;
}

// FNV-1a of a row: rows equal to what the unit already shows are skipped. Comparing the result
// instead of tracking writes, because a lambda that fills and redraws touches every pixel twice.
static uint32_t row_hash(const uint8_t *row) {
  uint32_t h = 2166136261u;
  for (int i = 0; i < WIDTH * 2; i++)
    h = (h ^ row[i]) * 16777619u;
  return h;
}

// A row as CMD_WRITE_RLE_16 into out (1 + WIDTH * 3 bytes), returns its length. Runs of 2+ equal
// pixels are 3 bytes, the pixels between them go out as literals. A busy row can come out longer
// than raw, it is sent as CMD_WRITE_RAW_16 then.
static size_t encode_rle(const uint8_t *px, uint8_t *out) {
  auto same = [px](int a, int b) { return px[a * 2] == px[b * 2] && px[a * 2 + 1] == px[b * 2 + 1]; };
  size_t n = 0;
  out[n++] = CMD_WRITE_RLE_16;
  int x = 0;
  while (x < WIDTH) {
    int run = 1;
    while (x + run < WIDTH && run < 255 && same(x, x + run))
      run++;
    if (run >= 2) {
      out[n++] = run;
      out[n++] = px[x * 2];
      out[n++] = px[x * 2 + 1];
      x += run;
      continue;
    }
    int lit = 1;  // up to the next run of 2
    while (x + lit < WIDTH && lit < 255 && !(x + lit + 1 < WIDTH && same(x + lit, x + lit + 1)))
      lit++;
    if (lit == 1) {
      out[n++] = 1;
    } else {
      out[n++] = 0;
      out[n++] = lit;
    }
    memcpy(out + n, px + x * 2, lit * 2);
    n += lit * 2;
    x += lit;
  }
  if (n > 1 + WIDTH * 2) {
    out[0] = CMD_WRITE_RAW_16;
    memcpy(out + 1, px, WIDTH * 2);
    n = 1 + WIDTH * 2;
  }
  return n;
}

void M5StackST7789V2::loop() {
  const uint32_t start = millis();
  while (this->connected_ && this->row_ < HEIGHT && millis() - start < LOOP_BUDGET_MS) {
    const int y = this->row_;
    const uint8_t *pixels = this->buffer_ + y * WIDTH * 2;
    const uint32_t hash = row_hash(pixels);
    if (hash == this->sent_hash_[y]) {
      this->row_++;
      continue;
    }
    uint8_t row[1 + WIDTH * 3];
    const size_t len = encode_rle(pixels, row);
    if (!this->wait_buffer_(2 + len / 8 + 1))
      return;
    const uint8_t window[] = {CMD_CASET, 0, WIDTH - 1, CMD_RASET, uint8_t(y), uint8_t(y)};
    if (this->write(window, sizeof(window)) != i2c::ERROR_OK || this->write(row, len) != i2c::ERROR_OK) {
      ESP_LOGW(TAG, "Unit LCD lost, reconnecting on the next update");
      this->connected_ = false;
      return;
    }
    this->sent_hash_[y] = hash;
    this->row_++;
  }
  if (!this->connected_ || this->row_ >= HEIGHT)
    this->disable_loop();
}

void M5StackST7789V2::draw_pixels_at(int x_start, int y_start, int w, int h, const uint8_t *ptr,
                                      display::ColorOrder order, display::ColorBitness bitness, bool big_endian,
                                      int x_offset, int y_offset, int x_pad) {
  if (this->buffer_ == nullptr)
    return;
  if (order != display::COLOR_ORDER_RGB || bitness != display::COLOR_BITNESS_565 ||
      this->get_rotation() != display::DISPLAY_ROTATION_0_DEGREES || this->is_clipping() || x_start < 0 ||
      y_start < 0 || w <= 0 || h <= 0 || x_start + w > WIDTH || y_start + h > HEIGHT) {
    display::Display::draw_pixels_at(x_start, y_start, w, h, ptr, order, bitness, big_endian, x_offset, y_offset,
                                     x_pad);
  } else {
    const size_t stride = size_t(x_offset + w + x_pad) * 2;
    for (int row = 0; row < h; row++) {
      const uint8_t *src = ptr + (y_offset + row) * stride + x_offset * 2;
      uint8_t *dst = this->buffer_ + ((y_start + row) * WIDTH + x_start) * 2;
      if (big_endian) {
        memcpy(dst, src, w * 2);
      } else {
        for (int i = 0; i < w * 2; i += 2) {
          dst[i] = src[i + 1];
          dst[i + 1] = src[i];
        }
      }
    }
  }
  // unchanged rows are skipped by their hash
  this->row_ = 0;
  if (this->connected_)
    this->enable_loop();
}

void HOT M5StackST7789V2::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT)
    return;
  const uint16_t c = display::ColorUtil::color_to_565(color);
  uint8_t *p = this->buffer_ + (y * WIDTH + x) * 2;
  p[0] = c >> 8;
  p[1] = c & 0xFF;
}

void M5StackST7789V2::dump_config() {
  LOG_DISPLAY("", "M5Stack ST7789V2 (Unit LCD)", this);
  LOG_I2C_DEVICE(this);
  ESP_LOGCONFIG(TAG, "  Brightness: %u", this->brightness_);
  LOG_UPDATE_INTERVAL(this);
}

}  // namespace esphome::m5stack_st7789v2

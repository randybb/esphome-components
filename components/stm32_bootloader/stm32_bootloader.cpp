#include "stm32_bootloader.h"

#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::stm32_bootloader {

ESPHOME_LOG_TAG(TAG, "stm32_bootloader");

// AN3155
static constexpr uint8_t SYNC = 0x7F;
static constexpr uint8_t ACK = 0x79;
static constexpr uint8_t NACK = 0x1F;
static constexpr uint8_t CMD_GET = 0x00;
static constexpr uint8_t CMD_GET_ID = 0x02;
static constexpr uint8_t CMD_READ = 0x11;
static constexpr uint8_t CMD_WRITE = 0x31;
static constexpr uint8_t CMD_ERASE = 0x43;
static constexpr uint8_t CMD_EXTENDED_ERASE = 0x44;

static constexpr uint32_t FLASH_BASE = 0x08000000;
static constexpr size_t CHUNK = 256;  // the most one Write/Read Memory command takes
static constexpr uint32_t ACK_TIMEOUT_MS = 1000;
static constexpr uint32_t ERASE_TIMEOUT_MS =
    10000;  // a 128 kB F4 sector takes up to 2 s at x8, more at x32 parallelism

// Same sector map as swd: 4 x 16 kB, 64 kB, then 128 kB
static bool f4_sector(uint32_t offset, uint32_t *index, uint32_t *start, uint32_t *size) {
  if (offset < 0x10000) {
    *index = offset / 0x4000;
    *size = 0x4000;
  } else if (offset < 0x20000) {
    *index = 4;
    *size = 0x10000;
  } else {
    *index = 5 + (offset - 0x20000) / 0x20000;
    *size = 0x20000;
  }
  *start = *index < 4 ? *index * 0x4000 : *index == 4 ? 0x10000 : 0x20000 + (*index - 5) * 0x20000;
  return *index < 12;
}

void STM32Bootloader::setup() {
  this->boot0_->setup();
  this->boot0_->digital_write(false);
  this->reset_->setup();
  this->reset_->digital_write(true);
}

void STM32Bootloader::dump_config() {
  ESP_LOGCONFIG(TAG, "STM32 bootloader:");
  LOG_PIN("  BOOT0 Pin: ", this->boot0_);
  LOG_PIN("  Reset Pin: ", this->reset_);
}

void STM32Bootloader::reset_target() {
  this->boot0_->digital_write(false);
  this->reset_->digital_write(false);
  delay(10);
  this->reset_->digital_write(true);
}

bool STM32Bootloader::read_bytes_(uint8_t *data, size_t len, uint32_t timeout_ms) {
  uint32_t start = millis();
  for (size_t i = 0; i < len;) {
    if (this->available()) {
      this->read_byte(&data[i++]);
      continue;
    }
    if (millis() - start > timeout_ms)
      return false;
    App.feed_wdt();
    yield();
  }
  return true;
}

bool STM32Bootloader::wait_ack_(uint32_t timeout_ms) {
  uint8_t b;
  if (!this->read_bytes_(&b, 1, timeout_ms)) {
    ESP_LOGE(TAG, "No response");
    return false;
  }
  if (b != ACK) {
    ESP_LOGE(TAG, "%s (0x%02X)", b == NACK ? "NACK" : "Unexpected response", b);
    return false;
  }
  return true;
}

bool STM32Bootloader::command_(uint8_t cmd) {
  uint8_t buf[2] = {cmd, uint8_t(~cmd)};
  this->write_array(buf, 2);
  return this->wait_ack_(ACK_TIMEOUT_MS);
}

bool STM32Bootloader::send_with_checksum_(const uint8_t *data, size_t len, uint32_t timeout_ms) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++)
    checksum ^= data[i];
  this->write_array(data, len);
  this->write_byte(checksum);
  return this->wait_ack_(timeout_ms);
}

bool STM32Bootloader::send_address_(uint32_t addr) {
  uint8_t buf[4] = {uint8_t(addr >> 24), uint8_t(addr >> 16), uint8_t(addr >> 8), uint8_t(addr)};
  return this->send_with_checksum_(buf, 4, ACK_TIMEOUT_MS);
}

bool STM32Bootloader::enter_() {
  this->parity_ = this->parent_->get_parity();
  this->parent_->set_parity(uart::UART_CONFIG_PARITY_EVEN);
  this->parent_->load_settings(false);

  this->boot0_->digital_write(true);
  this->reset_->digital_write(false);
  delay(10);
  this->reset_->digital_write(true);
  delay(100);  // ROM bootloader start-up
  this->discard_available();
  uint8_t b;

  // The bootloader detects the baud rate from 0x7F; NACK means it is synced already
  for (uint8_t i = 0; i < 3; i++) {
    this->write_byte(SYNC);
    if (this->read_bytes_(&b, 1, 500) && (b == ACK || b == NACK))
      return true;
  }
  ESP_LOGE(TAG,
           "Bootloader does not answer, check BOOT0, NRST, TX/RX and that the UART is one the bootloader listens on");
  return false;
}

void STM32Bootloader::leave_() {
  this->reset_target();
  this->parent_->set_parity(this->parity_);
  this->parent_->load_settings(false);
  delay(10);
  this->discard_available();
}

bool STM32Bootloader::get_() {
  if (!this->command_(CMD_GET))
    return false;
  uint8_t n, buf[32];
  if (!this->read_bytes_(&n, 1, ACK_TIMEOUT_MS) || n >= sizeof(buf) || !this->read_bytes_(buf, n + 1, ACK_TIMEOUT_MS))
    return false;
  // buf[0] is the bootloader version, the commands follow
  this->extended_erase_ = memchr(buf + 1, CMD_EXTENDED_ERASE, n) != nullptr;
  ESP_LOGI(TAG, "Bootloader v%u.%u", buf[0] >> 4, buf[0] & 0xF);
  return this->wait_ack_(ACK_TIMEOUT_MS);
}

bool STM32Bootloader::get_id_(uint16_t *pid) {
  if (!this->command_(CMD_GET_ID))
    return false;
  uint8_t buf[3];
  if (!this->read_bytes_(buf, 3, ACK_TIMEOUT_MS) || buf[0] != 1)
    return false;
  *pid = (buf[1] << 8) | buf[2];
  return this->wait_ack_(ACK_TIMEOUT_MS);
}

bool STM32Bootloader::erase_(uint32_t addr, uint32_t *next) {
  uint32_t index;
  if (this->family_ == Family::F1) {
    index = (addr - FLASH_BASE) / this->page_size_;
    *next = addr + this->page_size_;
  } else {
    uint32_t start, size;
    if (!f4_sector(addr - FLASH_BASE, &index, &start, &size)) {
      ESP_LOGE(TAG, "0x%08" PRIX32 " is beyond the supported 1 MB", addr);
      return false;
    }
    *next = FLASH_BASE + start + size;
  }
  if (this->extended_erase_) {
    uint8_t buf[4] = {0, 0, uint8_t(index >> 8), uint8_t(index)};  // N-1 = 0: one page/sector
    return this->command_(CMD_EXTENDED_ERASE) && this->send_with_checksum_(buf, 4, ERASE_TIMEOUT_MS);
  }
  uint8_t buf[2] = {0, uint8_t(index)};
  return this->command_(CMD_ERASE) && this->send_with_checksum_(buf, 2, ERASE_TIMEOUT_MS);
}

bool STM32Bootloader::write_(uint32_t addr, const uint8_t *data, size_t len) {
  if (!this->command_(CMD_WRITE) || !this->send_address_(addr))
    return false;
  uint8_t n = len - 1;
  uint8_t checksum = n;
  for (size_t i = 0; i < len; i++)
    checksum ^= data[i];
  this->write_byte(n);
  this->write_array(data, len);
  this->write_byte(checksum);
  return this->wait_ack_(ACK_TIMEOUT_MS);
}

bool STM32Bootloader::verify_(uint32_t addr, const uint8_t *data, size_t len) {
  if (!this->command_(CMD_READ) || !this->send_address_(addr))
    return false;
  uint8_t n[2] = {uint8_t(len - 1), uint8_t(~(len - 1))};
  this->write_array(n, 2);
  uint8_t check[CHUNK];
  if (!this->wait_ack_(ACK_TIMEOUT_MS) || !this->read_bytes_(check, len, ACK_TIMEOUT_MS))
    return false;
  if (memcmp(check, data, len) != 0) {
    ESP_LOGE(TAG, "Verify failed at 0x%08" PRIX32, addr);
    return false;
  }
  return true;
}

bool STM32Bootloader::download_(const std::string &url, uint32_t address) {
  auto container = this->http_->get(url);
  if (container == nullptr || container->status_code != 200) {
    ESP_LOGE(TAG, "Download of %s failed (HTTP %d)", url.c_str(), container ? container->status_code : -1);
    if (container != nullptr)
      container->end();
    return false;
  }
  uint8_t buf[CHUNK];
  size_t fill = 0;
  uint32_t addr = address;
  uint32_t erased = address;
  uint32_t last_data = millis();
  bool ok = true;
  bool done = false;
  while (ok && !done) {
    App.feed_wdt();
    int n = container->read(buf + fill, CHUNK - fill);
    auto result =
        http_request::http_read_loop_result(n, last_data, this->http_->get_timeout(), container->is_read_complete());
    if (result == http_request::HttpReadLoopResult::RETRY)
      continue;
    if (result == http_request::HttpReadLoopResult::ERROR || result == http_request::HttpReadLoopResult::TIMEOUT) {
      ESP_LOGE(TAG, "Download failed after %zu bytes", container->get_bytes_read());
      ok = false;
      break;
    }
    if (result == http_request::HttpReadLoopResult::DATA)
      fill += n;
    done = result == http_request::HttpReadLoopResult::COMPLETE ||
           (container->content_length > 0 && container->get_bytes_read() >= container->content_length);
    if (fill < CHUNK && !(done && fill > 0))
      continue;
    while (fill % 4)  // Write Memory takes multiples of 4 bytes
      buf[fill++] = 0xFF;
    while (ok && erased < addr + fill)
      ok = this->erase_(erased, &erased);
    ok = ok && this->write_(addr, buf, fill) && this->verify_(addr, buf, fill);
    addr += fill;
    fill = 0;
    if (ok && (addr - address) % 0x4000 == 0)
      ESP_LOGI(TAG, "Flashed %" PRIu32 " kB", (addr - address) / 1024);
  }
  container->end();
  if (ok)
    ESP_LOGI(TAG, "Flashed %" PRIu32 " bytes at 0x%08" PRIX32, addr - address, address);
  return ok;
}

bool STM32Bootloader::flash(const std::string &url, uint32_t address) {
  ESP_LOGI(TAG, "Flashing %s to 0x%08" PRIX32, url.c_str(), address);
  uint16_t pid = 0;
  bool ok = this->enter_() && this->get_() && this->get_id_(&pid);
  if (ok) {
    switch (pid) {
      case 0x412:  // F10x low density
      case 0x410:  // F10x medium density
        this->family_ = Family::F1;
        this->page_size_ = 1024;
        break;
      case 0x414:  // F10x high density
      case 0x418:  // F105/F107
      case 0x430:  // F10x XL density, ponytail: bank 1 only (512 kB), page numbers above it untested
        this->family_ = Family::F1;
        this->page_size_ = 2048;
        break;
      case 0x413:  // F405/F407
      case 0x419:  // F42x/F43x, ponytail: first 1 MB only
      case 0x421:  // F446
      case 0x423:  // F401xB/C
      case 0x431:  // F411
      case 0x433:  // F401xD/E
      case 0x434:  // F469
      case 0x441:  // F412
      case 0x458:  // F410
      case 0x463:  // F413
        this->family_ = Family::F4;
        break;
      default:
        this->family_ = Family::NONE;
        break;
    }
    bool aligned;
    if (this->family_ == Family::F1) {
      aligned = address >= FLASH_BASE && (address - FLASH_BASE) % this->page_size_ == 0;
    } else {
      uint32_t index, start, size;
      aligned = address >= FLASH_BASE && f4_sector(address - FLASH_BASE, &index, &start, &size) &&
                start == address - FLASH_BASE;
    }
    ESP_LOGI(TAG, "Device 0x%03X", pid);
    if (this->family_ == Family::NONE) {
      ESP_LOGE(TAG, "Unsupported device 0x%03X", pid);
      ok = false;
    } else if (!aligned) {
      ESP_LOGE(TAG, "0x%08" PRIX32 " does not start a flash page/sector", address);
      ok = false;
    }
  }
  ok = ok && this->download_(url, address);
  this->leave_();
  return ok;
}

}  // namespace esphome::stm32_bootloader

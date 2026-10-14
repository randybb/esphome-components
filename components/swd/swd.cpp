#include "swd.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::swd {

ESPHOME_LOG_TAG(TAG, "swd");
ESPHOME_LOG_TAG(RTT_TAG, "rtt");

static constexpr uint8_t ACK_OK = 1;
static constexpr uint8_t ACK_WAIT = 2;
static constexpr uint8_t ACK_FAULT = 4;

// DP registers
static constexpr uint8_t DP_IDCODE = 0x0;  // read
static constexpr uint8_t DP_ABORT = 0x0;   // write
static constexpr uint8_t DP_CTRL_STAT = 0x4;
static constexpr uint8_t DP_SELECT = 0x8;
static constexpr uint8_t DP_RDBUFF = 0xC;
// MEM-AP registers (bank 0)
static constexpr uint8_t AP_CSW = 0x0;
static constexpr uint8_t AP_TAR = 0x4;
static constexpr uint8_t AP_DRW = 0xC;

static constexpr uint32_t ABORT_CLEAR_ALL = 0x1E;
static constexpr uint32_t PWRUP_REQ = 0x50000000;  // CSYSPWRUPREQ | CDBGPWRUPREQ
static constexpr uint32_t PWRUP_ACK = 0xA0000000;
static constexpr uint32_t CSW_BASE = 0x23000000;  // master type debug, privileged
static constexpr uint32_t CSW_INCREMENT = 0x10;   // TAR auto-increment, wraps at 1 kB boundaries
static constexpr uint32_t AIRCR = 0xE000ED0C;
static constexpr uint32_t AIRCR_SYSRESETREQ = 0x05FA0004;
static constexpr uint32_t DBGMCU_CR = 0xE0042004;  // STM32
static constexpr uint32_t DBGMCU_SLEEP_STOP_STANDBY = 0x7;
static constexpr uint32_t DBGMCU_IDCODE = 0xE0042000;  // STM32
static constexpr uint32_t DHCSR = 0xE000EDF0;
static constexpr uint32_t DHCSR_HALT = 0xA05F0003;  // DBGKEY | C_HALT | C_DEBUGEN
static constexpr uint32_t DHCSR_RUN = 0xA05F0000;
static constexpr uint32_t DHCSR_S_HALT = 1 << 17;
static constexpr uint32_t DEMCR = 0xE000EDFC;
static constexpr uint32_t DEMCR_VC_CORERESET = 1;

// STM32 flash
static constexpr uint32_t FLASH_BASE = 0x08000000;
static constexpr uint32_t FLASH_KEY1 = 0x45670123;
static constexpr uint32_t FLASH_KEY2 = 0xCDEF89AB;
static constexpr uint32_t F1_KEYR = 0x40022004;
static constexpr uint32_t F1_SR = 0x4002200C;
static constexpr uint32_t F1_CR = 0x40022010;
static constexpr uint32_t F1_AR = 0x40022014;
static constexpr uint32_t F1_SR_BSY = 1 << 0;
static constexpr uint32_t F1_SR_ERRORS = 0x14;  // PGERR, WRPRTERR
static constexpr uint32_t F1_CR_PG = 1 << 0;
static constexpr uint32_t F1_CR_PER = 1 << 1;
static constexpr uint32_t F1_CR_STRT = 1 << 6;
static constexpr uint32_t F1_CR_LOCK = 1 << 7;
static constexpr uint32_t F4_KEYR = 0x40023C04;
static constexpr uint32_t F4_SR = 0x40023C0C;
static constexpr uint32_t F4_CR = 0x40023C10;
static constexpr uint32_t F4_SR_BSY = 1 << 16;
static constexpr uint32_t F4_SR_ERRORS = 0x1F2;  // RDERR, PGSERR, PGPERR, PGAERR, WRPERR, OPERR
static constexpr uint32_t F4_CR_PG = 1 << 0;
static constexpr uint32_t F4_CR_SER = 1 << 1;
static constexpr uint32_t F4_CR_PSIZE_32 = 2 << 8;
static constexpr uint32_t F4_CR_STRT = 1 << 16;
static constexpr uint32_t F4_CR_LOCK = 1u << 31;
static constexpr size_t FLASH_CHUNK = 1024;

// "SEGGER RTT\0\0" as little endian words
static constexpr uint32_t RTT_ID[3] = {0x47474553, 0x52205245, 0x00005454};
static constexpr uint32_t RTT_UP0 = 24;  // acID[16], MaxNumUpBuffers, MaxNumDownBuffers
static constexpr uint32_t RTT_MAX_BYTES_PER_LOOP = 256;
static constexpr uint32_t SCAN_WORDS_PER_LOOP = 64;
static constexpr size_t MAX_LINE = 200;

void SWD::setup() {
  this->swclk_->setup();
  this->swdio_->setup();
  this->swclk_->digital_write(true);
  this->swdio_->digital_write(true);
}

void SWD::dump_config() {
  ESP_LOGCONFIG(TAG,
                "SWD:\n"
                "  RAM: 0x%08" PRIX32 " - 0x%08" PRIX32 "\n"
                "  Clock delay: %" PRIu32 " us\n"
                "  IDCODE: 0x%08" PRIX32,
                this->ram_start_, this->ram_end_, this->clock_delay_us_, this->idcode_);
  LOG_PIN("  SWCLK Pin: ", this->swclk_);
  LOG_PIN("  SWDIO Pin: ", this->swdio_);
}

void SWD::loop() {
  switch (this->state_) {
    case State::DISCONNECTED:
      if (int32_t(millis() - this->next_try_ms_) < 0)
        return;
      if (!this->connect_()) {
        if (!this->warned_)
          ESP_LOGW(TAG, "No target");
        this->warned_ = true;
        this->next_try_ms_ = millis() + 1000;
        return;
      }
      this->warned_ = false;
      ESP_LOGI(TAG, "Connected, IDCODE 0x%08" PRIX32 ", looking for RTT", this->idcode_);
      this->scan_addr_ = this->ram_start_;
      this->state_ = State::SCANNING;
      return;
    case State::SCANNING:
      if (int32_t(millis() - this->next_try_ms_) >= 0 && !this->scan_())
        this->disconnect_(1000);
      return;
    case State::RTT:
      if (!this->poll_rtt_())
        this->disconnect_(1000);
      return;
  }
}

void SWD::reset_target() {
  if (this->state_ == State::DISCONNECTED)
    return;
  this->write_word(AIRCR, AIRCR_SYSRESETREQ);
  ESP_LOGI(TAG, "Target reset");
  this->disconnect_(500);
}

void SWD::disconnect_(uint32_t retry_ms) {
  this->state_ = State::DISCONNECTED;
  this->next_try_ms_ = millis() + retry_ms;
  this->line_.clear();
}

// Bit level. The target samples SWDIO on the rising edge of SWCLK and changes it after the rising edge.

void SWD::clock_() {
  this->swclk_->digital_write(false);
  delayMicroseconds(this->clock_delay_us_);
  this->swclk_->digital_write(true);
  delayMicroseconds(this->clock_delay_us_);
}

void SWD::write_bits_(uint32_t value, uint8_t count) {
  for (uint8_t i = 0; i < count; i++, value >>= 1) {
    this->swdio_->digital_write(value & 1);
    this->clock_();
  }
}

uint32_t SWD::read_bits_(uint8_t count) {
  uint32_t value = 0;
  for (uint8_t i = 0; i < count; i++) {
    this->swclk_->digital_write(false);
    delayMicroseconds(this->clock_delay_us_);
    value |= uint32_t(this->swdio_->digital_read()) << i;
    this->swclk_->digital_write(true);
    delayMicroseconds(this->clock_delay_us_);
  }
  return value;
}

void SWD::swdio_output_(bool output) {
  if (output) {
    this->swdio_->pin_mode(gpio::FLAG_OUTPUT);
    this->swdio_->digital_write(true);
  } else {
    this->swdio_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
  }
}

// Packet level

bool SWD::transfer_(bool ap, bool read, uint8_t addr, uint32_t *data) {
  // start, APnDP, RnW, A[2:3], parity, stop, park
  uint8_t req = 0x81 | (ap << 1) | (read << 2) | ((addr & 0xC) << 1);
  if (__builtin_parity(req & 0x1E))
    req |= 0x20;

  for (uint8_t retry = 0; retry < 100; retry++) {
    this->write_bits_(req, 8);
    this->swdio_output_(false);
    this->clock_();  // turnaround
    uint8_t ack = this->read_bits_(3);

    if (ack == ACK_OK) {
      if (read) {
        uint32_t value = this->read_bits_(32);
        bool parity = this->read_bits_(1);
        this->clock_();  // turnaround
        this->swdio_output_(true);
        this->write_bits_(0, 8);  // idle
        if (parity != bool(__builtin_parity(value)))
          return false;
        *data = value;
      } else {
        this->clock_();  // turnaround
        this->swdio_output_(true);
        this->write_bits_(*data, 32);
        this->write_bits_(__builtin_parity(*data), 1);
        this->write_bits_(0, 8);  // idle
      }
      return true;
    }

    this->clock_();  // turnaround
    this->swdio_output_(true);
    this->write_bits_(0, 8);
    if (ack == ACK_FAULT) {
      uint32_t clear = ABORT_CLEAR_ALL;
      this->transfer_(false, false, DP_ABORT, &clear);  // never FAULTs itself
    }
    if (ack != ACK_WAIT)
      return false;  // FAULT, or no target driving the line
  }
  return false;
}

bool SWD::connect_() {
  this->swdio_output_(true);
  // Line reset, JTAG to SWD switch, line reset, idle
  this->write_bits_(0xFFFFFFFF, 32);
  this->write_bits_(0xFFFFFFFF, 32);
  this->write_bits_(0xE79E, 16);
  this->write_bits_(0xFFFFFFFF, 32);
  this->write_bits_(0xFFFFFFFF, 32);
  this->write_bits_(0, 8);

  if (!this->transfer_(false, true, DP_IDCODE, &this->idcode_))
    return false;
  uint32_t value = ABORT_CLEAR_ALL;
  this->transfer_(false, false, DP_ABORT, &value);
  value = PWRUP_REQ;
  if (!this->transfer_(false, false, DP_CTRL_STAT, &value))
    return false;
  for (uint8_t i = 0;; i++) {
    if (!this->transfer_(false, true, DP_CTRL_STAT, &value))
      return false;
    if ((value & PWRUP_ACK) == PWRUP_ACK)
      break;
    if (i == 100)
      return false;
  }
  value = 0;  // AP 0, bank 0
  if (!this->transfer_(false, false, DP_SELECT, &value))
    return false;
  this->csw_ = 0;
  if (!this->set_csw_(4, false))
    return false;
  // Keeps the debug port alive while the target sleeps. Not an STM32: FAULTs, which is cleared.
  this->write_word(DBGMCU_CR, DBGMCU_SLEEP_STOP_STANDBY);
  return true;
}

bool SWD::set_csw_(uint8_t size, bool increment) {
  uint32_t csw = CSW_BASE | (size == 1 ? 0 : size == 2 ? 1 : 2) | (increment ? CSW_INCREMENT : 0);
  if (csw == this->csw_)
    return true;
  this->csw_ = 0;
  if (!this->transfer_(true, false, AP_CSW, &csw))
    return false;
  this->csw_ = csw;
  return true;
}

// ponytail: one word per TAR write, 2 extra transfers per word; use read_block_() if RTT throughput matters.
bool SWD::read_word(uint32_t addr, uint32_t *value) {
  uint32_t dummy;
  return this->set_csw_(4, false) && this->transfer_(true, false, AP_TAR, &addr) &&
         this->transfer_(true, true, AP_DRW, &dummy) &&
         this->transfer_(false, true, DP_RDBUFF, value);  // AP reads are posted
}

bool SWD::write_word(uint32_t addr, uint32_t value) {
  return this->set_csw_(4, false) && this->transfer_(true, false, AP_TAR, &addr) &&
         this->transfer_(true, false, AP_DRW, &value);
}

/// Writes `len` bytes in accesses of `size` (2 or 4) bytes, `len` a multiple of `size`.
bool SWD::write_block_(uint32_t addr, const uint8_t *data, size_t len, uint8_t size) {
  if (!this->set_csw_(size, true))
    return false;
  for (size_t i = 0; i < len; i += size) {
    uint32_t a = addr + i;
    if ((i == 0 || (a & 0x3FF) == 0) && !this->transfer_(true, false, AP_TAR, &a))
      return false;
    uint32_t value = 0;
    memcpy(&value, data + i, size);
    value <<= (a & 3) * 8;  // byte lanes
    if (!this->transfer_(true, false, AP_DRW, &value))
      return false;
  }
  return true;
}

/// Reads `len` bytes, `addr` and `len` word aligned.
bool SWD::read_block_(uint32_t addr, uint8_t *data, size_t len) {
  if (!this->set_csw_(4, true))
    return false;
  for (size_t i = 0; i < len;) {
    uint32_t a = addr + i;
    size_t n = std::min(len - i, size_t(0x400 - (a & 0x3FF)));
    uint32_t value;
    // Posted reads: each DRW read returns the previous one, RDBUFF the last
    if (!this->transfer_(true, false, AP_TAR, &a) || !this->transfer_(true, true, AP_DRW, &value))
      return false;
    for (size_t j = 4; j < n; j += 4) {
      if (!this->transfer_(true, true, AP_DRW, &value))
        return false;
      memcpy(data + i + j - 4, &value, 4);
    }
    if (!this->transfer_(false, true, DP_RDBUFF, &value))
      return false;
    memcpy(data + i + n - 4, &value, 4);
    i += n;
  }
  return true;
}

#ifdef USE_SWD_FLASH

/// Sector of an STM32F4 flash offset: 4x 16 kB, 64 kB, then 128 kB each. False above 1 MB.
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

bool SWD::halt_after_reset_() {
  // Halt on the reset vector, so the target firmware neither runs nor reconfigures clocks or the SWD pins
  if (!this->write_word(DEMCR, DEMCR_VC_CORERESET) || !this->write_word(DHCSR, DHCSR_HALT))
    return false;
  this->write_word(AIRCR, AIRCR_SYSRESETREQ);  // the reset may drop the response
  for (uint8_t i = 0; i < 20; i++) {
    delay(10);
    uint32_t dhcsr;
    if (this->read_word(DHCSR, &dhcsr) && (dhcsr & DHCSR_S_HALT))
      return this->write_word(DEMCR, 0);
    this->connect_();
  }
  return false;
}

bool SWD::flash_wait_(uint32_t timeout_ms) {
  bool f1 = this->family_ == Family::F1;
  uint32_t start = millis();
  uint32_t sr;
  while (true) {
    if (!this->read_word(f1 ? F1_SR : F4_SR, &sr))
      return false;
    if (!(sr & (f1 ? F1_SR_BSY : F4_SR_BSY)))
      break;
    if (millis() - start > timeout_ms) {
      ESP_LOGE(TAG, "Flash busy for over %" PRIu32 " ms", timeout_ms);
      return false;
    }
    App.feed_wdt();
  }
  uint32_t errors = sr & (f1 ? F1_SR_ERRORS : F4_SR_ERRORS);
  if (errors) {
    ESP_LOGE(TAG, "Flash error, SR 0x%08" PRIX32, sr);
    this->write_word(f1 ? F1_SR : F4_SR, errors);  // write 1 to clear
    return false;
  }
  return true;
}

/// Erases the page/sector containing `addr`, `next` is where the following one starts.
bool SWD::flash_erase_(uint32_t addr, uint32_t *next) {
  bool ok;
  if (this->family_ == Family::F1) {
    ok = this->write_word(F1_CR, F1_CR_PER) && this->write_word(F1_AR, addr) &&
         this->write_word(F1_CR, F1_CR_PER | F1_CR_STRT) && this->flash_wait_(100);
    this->write_word(F1_CR, 0);
    *next = addr - addr % this->page_size_ + this->page_size_;
  } else {
    uint32_t index, start, size;
    if (!f4_sector(addr - FLASH_BASE, &index, &start, &size)) {
      ESP_LOGE(TAG, "0x%08" PRIX32 " is beyond the supported 1 MB", addr);
      return false;
    }
    uint32_t cr = F4_CR_SER | (index << 3) | F4_CR_PSIZE_32;
    ok = this->write_word(F4_CR, cr) && this->write_word(F4_CR, cr | F4_CR_STRT) && this->flash_wait_(5000);
    this->write_word(F4_CR, 0);
    *next = FLASH_BASE + start + size;
  }
  if (ok)
    ESP_LOGD(TAG, "Erased up to 0x%08" PRIX32, *next);
  return ok;
}

/// Programs and verifies `len` bytes, a multiple of 4.
bool SWD::flash_program_(uint32_t addr, const uint8_t *data, size_t len) {
  bool f1 = this->family_ == Family::F1;
  // The bus stalls (SWD WAIT) while a half-word/word is programmed, so no BSY polling in between
  bool ok = this->write_word(f1 ? F1_CR : F4_CR, f1 ? F1_CR_PG : F4_CR_PG | F4_CR_PSIZE_32) &&
            this->write_block_(addr, data, len, f1 ? 2 : 4) && this->flash_wait_(100);
  this->write_word(f1 ? F1_CR : F4_CR, 0);
  if (!ok)
    return false;
  uint8_t check[FLASH_CHUNK];
  if (!this->read_block_(addr, check, len))
    return false;
  if (memcmp(check, data, len) != 0) {
    ESP_LOGE(TAG, "Verify failed at 0x%08" PRIX32, addr);
    return false;
  }
  return true;
}

bool SWD::flash_download_(const std::string &url, uint32_t address) {
  auto container = this->http_->get(url);
  if (container == nullptr || container->status_code != 200) {
    ESP_LOGE(TAG, "Download of %s failed (HTTP %d)", url.c_str(), container ? container->status_code : -1);
    if (container != nullptr)
      container->end();
    return false;
  }
  uint8_t buf[FLASH_CHUNK];
  size_t fill = 0;
  uint32_t addr = address;
  uint32_t erased = address;
  uint32_t last_data = millis();
  bool ok = true;
  bool done = false;
  while (ok && !done) {
    App.feed_wdt();
    int n = container->read(buf + fill, FLASH_CHUNK - fill);
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
    if (fill < FLASH_CHUNK && !(done && fill > 0))
      continue;
    while (fill % 4)
      buf[fill++] = 0xFF;
    while (ok && erased < addr + fill)
      ok = this->flash_erase_(erased, &erased);
    ok = ok && this->flash_program_(addr, buf, fill);
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

bool SWD::flash(const std::string &url, uint32_t address) {
  if (this->http_ == nullptr)
    return false;
  ESP_LOGI(TAG, "Flashing %s to 0x%08" PRIX32, url.c_str(), address);
  uint32_t idcode;
  if (!this->connect_() || !this->halt_after_reset_() || !this->read_word(DBGMCU_IDCODE, &idcode)) {
    ESP_LOGE(TAG, "Cannot halt the target");
    this->disconnect_(1000);
    return false;
  }

  uint16_t dev_id = idcode & 0xFFF;
  bool aligned;
  switch (dev_id) {
    case 0x412:  // F10x low density
    case 0x410:  // F10x medium density
      this->family_ = Family::F1;
      this->page_size_ = 1024;
      break;
    case 0x414:  // F10x high density
    case 0x418:  // F105/F107
    case 0x430:  // F10x XL density, ponytail: bank 1 only (512 kB), bank 2 has its own registers
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
  if (this->family_ == Family::F1) {
    aligned = address % this->page_size_ == 0;
  } else {
    uint32_t index, start, size;
    aligned = f4_sector(address - FLASH_BASE, &index, &start, &size) && start == address - FLASH_BASE;
  }

  bool ok = false;
  if (this->family_ == Family::NONE) {
    ESP_LOGE(TAG, "Unsupported device 0x%03X", dev_id);
  } else if (address < FLASH_BASE || !aligned) {
    ESP_LOGE(TAG, "0x%08" PRIX32 " does not start a flash page/sector", address);
  } else {
    bool f1 = this->family_ == Family::F1;
    uint32_t cr;
    ok = this->read_word(f1 ? F1_CR : F4_CR, &cr);
    if (ok && (cr & (f1 ? F1_CR_LOCK : F4_CR_LOCK)))
      ok = this->write_word(f1 ? F1_KEYR : F4_KEYR, FLASH_KEY1) && this->write_word(f1 ? F1_KEYR : F4_KEYR, FLASH_KEY2);
    this->write_word(f1 ? F1_SR : F4_SR, f1 ? F1_SR_ERRORS : F4_SR_ERRORS);  // stale errors block programming
    ESP_LOGI(TAG, "Device 0x%03X, flash %s", dev_id, ok ? "unlocked" : "locked");
    ok = ok && this->flash_download_(url, address);
    this->write_word(f1 ? F1_CR : F4_CR, f1 ? F1_CR_LOCK : F4_CR_LOCK);
  }

  this->write_word(DHCSR, DHCSR_RUN);
  this->write_word(AIRCR, AIRCR_SYSRESETREQ);
  this->disconnect_(500);
  return ok;
}

#endif  // USE_SWD_FLASH

// RTT

bool SWD::scan_() {
  for (uint32_t i = 0; i < SCAN_WORDS_PER_LOOP; i++, this->scan_addr_ += 4) {
    if (this->scan_addr_ + sizeof(RTT_ID) > this->ram_end_) {
      // Not set up yet (the target calls SEGGER_RTT_Init lazily), try again later
      this->scan_addr_ = this->ram_start_;
      this->next_try_ms_ = millis() + 5000;
      ESP_LOGD(TAG, "No RTT control block found");
      return true;
    }
    uint32_t word;
    if (!this->read_word(this->scan_addr_, &word))
      return false;
    if (word != RTT_ID[0])
      continue;
    uint32_t id1, id2;
    if (!this->read_word(this->scan_addr_ + 4, &id1) || !this->read_word(this->scan_addr_ + 8, &id2))
      return false;
    if (id1 != RTT_ID[1] || id2 != RTT_ID[2])
      continue;

    uint32_t up0 = this->scan_addr_ + RTT_UP0;
    if (!this->read_word(up0 + 4, &this->rtt_buffer_) || !this->read_word(up0 + 8, &this->rtt_size_))
      return false;
    if (this->rtt_size_ == 0)
      continue;
    this->rtt_cb_ = this->scan_addr_;
    ESP_LOGI(TAG, "RTT control block at 0x%08" PRIX32 ", up-buffer 0: %" PRIu32 " bytes at 0x%08" PRIX32, this->rtt_cb_,
             this->rtt_size_, this->rtt_buffer_);
    this->state_ = State::RTT;
    return true;
  }
  return true;
}

bool SWD::poll_rtt_() {
  uint32_t up0 = this->rtt_cb_ + RTT_UP0;
  uint32_t id, wr, rd;
  if (!this->read_word(this->rtt_cb_, &id) || !this->read_word(up0 + 12, &wr) || !this->read_word(up0 + 16, &rd))
    return false;
  if (id != RTT_ID[0] || wr >= this->rtt_size_ || rd >= this->rtt_size_) {
    ESP_LOGI(TAG, "RTT control block gone, looking again");
    this->scan_addr_ = this->ram_start_;
    this->state_ = State::SCANNING;
    return true;
  }

  uint32_t budget = RTT_MAX_BYTES_PER_LOOP;
  while (rd != wr && budget > 0) {
    uint32_t end = std::min(wr > rd ? wr : this->rtt_size_, rd + budget);
    budget -= end - rd;
    while (rd < end) {
      uint32_t addr = this->rtt_buffer_ + rd;
      uint32_t word;
      if (!this->read_word(addr & ~3u, &word))
        return false;
      for (uint32_t shift = (addr & 3) * 8; shift < 32 && rd < end; shift += 8, rd++)
        this->put_char_(char(word >> shift));
    }
    if (rd == this->rtt_size_)
      rd = 0;
    if (!this->write_word(up0 + 16, rd))
      return false;
  }
  return true;
}

void SWD::put_char_(char c) {
  if (c == '\r')
    return;
  if (c != '\n') {
    this->line_ += c;
    if (this->line_.size() < MAX_LINE)
      return;
  }
  ESP_LOGI(RTT_TAG, "%s", this->line_.c_str());
  this->line_.clear();
}

}  // namespace esphome::swd

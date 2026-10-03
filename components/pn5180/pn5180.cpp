#include "pn5180.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iterator>

#include "esphome/core/log.h"

namespace esphome::pn5180 {

static const char *const TAG = "pn5180";

// Host commands
static constexpr uint8_t CMD_WRITE_REGISTER = 0x00;
static constexpr uint8_t CMD_WRITE_REGISTER_OR_MASK = 0x01;
static constexpr uint8_t CMD_WRITE_REGISTER_AND_MASK = 0x02;
static constexpr uint8_t CMD_READ_REGISTER = 0x04;
static constexpr uint8_t CMD_READ_EEPROM = 0x07;
static constexpr uint8_t CMD_SEND_DATA = 0x09;
static constexpr uint8_t CMD_READ_DATA = 0x0A;
static constexpr uint8_t CMD_LOAD_RF_CONFIG = 0x11;
static constexpr uint8_t CMD_RF_ON = 0x16;

// Registers
static constexpr uint8_t REG_SYSTEM_CONFIG = 0x00;
static constexpr uint8_t REG_IRQ_ENABLE = 0x01;
static constexpr uint8_t REG_IRQ_STATUS = 0x02;
static constexpr uint8_t REG_IRQ_CLEAR = 0x03;
static constexpr uint8_t REG_RX_STATUS = 0x13;
static constexpr uint8_t REG_RF_STATUS = 0x1D;

static constexpr uint8_t EEPROM_FIRMWARE_VERSION = 0x12;
static constexpr uint8_t EEPROM_IRQ_PIN_CONFIG = 0x1A;  // bit 0: IRQ pin active high

static constexpr uint32_t IRQ_RX = 1 << 0;
static constexpr uint32_t IRQ_IDLE = 1 << 2;
static constexpr uint32_t IRQ_TX_RFON = 1 << 9;
static constexpr uint32_t IRQ_GENERAL_ERROR = 1 << 17;

static constexpr uint32_t RX_BYTES_MASK = 0x1FF;
static constexpr uint32_t TRANSCEIVE_STATE_WAIT_TRANSMIT = 1;

// ISO 15693-3, ASK 100%, 26 kbps
static constexpr uint8_t RF_CONFIG_ISO15693_TX = 0x0D;
static constexpr uint8_t RF_CONFIG_ISO15693_RX = 0x8D;
static constexpr uint8_t ISO15693_FLAGS_INVENTORY = 0x26;  // high data rate, inventory, one slot
static constexpr uint8_t ISO15693_FLAGS_ADDRESSED = 0x22;  // high data rate, addressed
static constexpr uint8_t ISO15693_INVENTORY = 0x01;
static constexpr uint8_t ISO15693_READ_SINGLE_BLOCK = 0x20;
static constexpr uint8_t ISO15693_READ_MULTIPLE_BLOCKS = 0x23;
static constexpr uint8_t ISO15693_WRITE_SINGLE_BLOCK = 0x21;
static constexpr uint8_t ISO15693_ERROR_FLAG = 0x01;

static constexpr uint32_t BUSY_TIMEOUT_MS = 100;
static constexpr uint32_t RX_TIMEOUT_ID = 0;
static constexpr uint32_t RX_TIMEOUT_MS = 50;
static constexpr uint8_t BLOCKS_PER_READ = 8;
static constexpr size_t MAX_TAG_MEMORY = 2048;

// material_type enum of the spec, indexed by key
static const char *const MATERIAL_TYPES[] = {
    "PLA",  "PETG", "TPU", "ABS",  "ASA",  "PC",   "PCTG", "PP",  "PA6",  "PA11", "PA12", "PA66", "CPE",  "TPE", "HIPS",
    "PHA",  "PET",  "PEI", "PBT",  "PVB",  "PVA",  "PEKK", "PEEK", "BVOH", "TPC", "PPS",  "PPSU", "PVC",  "PEBA", "PVDF",
    "PPA",  "PCL",  "PES", "PMMA", "POM",  "PPE",  "PS",   "PSU", "TPI",  "SBS",  "OBC",  "EVA",  "PA612",
};
static constexpr uint32_t KEY_MATERIAL_TYPE = 9;
static constexpr uint32_t KEY_PRIMARY_COLOR = 19;
static constexpr uint32_t KEY_SECONDARY_COLOR_4 = 24;
static constexpr uint32_t KEY_NOMINAL_WEIGHT = 16;
static constexpr uint32_t KEY_ACTUAL_WEIGHT = 17;
static constexpr uint32_t KEY_AUX_CONSUMED_WEIGHT = 0;

void PN5180::setup() {
  this->spi_setup();
  this->busy_pin_->setup();
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(10);
  }
  this->wait_irq_(IRQ_IDLE, 100);  // startup done

  uint8_t fw[2];
  if (!this->command_({CMD_READ_EEPROM, EEPROM_FIRMWARE_VERSION, sizeof(fw)}, fw, sizeof(fw)) ||
      (fw[0] == fw[1] && (fw[0] == 0x00 || fw[0] == 0xFF))) {
    ESP_LOGE(TAG, "No response, check SPI, BUSY and the 5 V supply");
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "Firmware %u.%u", fw[1], fw[0]);

  if (this->irq_pin_ != nullptr) {
    uint8_t irq_config;
    if (!this->command_({CMD_READ_EEPROM, EEPROM_IRQ_PIN_CONFIG, 1}, &irq_config, 1) ||
        !this->write_register_(CMD_WRITE_REGISTER, REG_IRQ_ENABLE, IRQ_RX | IRQ_GENERAL_ERROR)) {
      this->mark_failed();
      return;
    }
    this->irq_active_high_ = irq_config & 0x01;
    this->irq_pin_->setup();
    this->irq_pin_->attach_interrupt(&PN5180::gpio_intr, this,
                                     this->irq_active_high_ ? gpio::INTERRUPT_RISING_EDGE
                                                            : gpio::INTERRUPT_FALLING_EDGE);
  }

  if (!this->command_({CMD_LOAD_RF_CONFIG, RF_CONFIG_ISO15693_TX, RF_CONFIG_ISO15693_RX}) || !this->rf_on_()) {
    ESP_LOGE(TAG, "RF field did not turn on");
    this->mark_failed();
  }
  this->disable_loop();  // runs only while waiting for a tag to answer
}

void IRAM_ATTR PN5180::gpio_intr(PN5180 *arg) { arg->enable_loop_soon_any_context(); }

void PN5180::dump_config() {
  ESP_LOGCONFIG(TAG, "PN5180 (ISO 15693, OpenPrintTag):");
  LOG_PIN("  BUSY Pin: ", this->busy_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  IRQ Pin: ", this->irq_pin_);
  LOG_UPDATE_INTERVAL(this);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup failed");
}

void PN5180::update() {
  if (this->state_ == State::IDLE)
    this->send_(State::INVENTORY, {ISO15693_FLAGS_INVENTORY, ISO15693_INVENTORY, 0x00});
}

// Waits for the tag to answer: with the IRQ pin only woken by the ISR, without it polls IRQ_STATUS
void PN5180::loop() {
  if (this->state_ == State::IDLE) {
    this->disable_loop();
    return;
  }
  if (this->irq_pin_ != nullptr && this->irq_pin_->digital_read() != this->irq_active_high_) {
    this->disable_loop();
    return;
  }
  uint32_t irq;
  if (!this->read_register_(REG_IRQ_STATUS, irq) || (irq & IRQ_GENERAL_ERROR)) {
    this->cancel_timeout(RX_TIMEOUT_ID);
    this->on_response_(nullptr);
  } else if (irq & IRQ_RX) {
    this->cancel_timeout(RX_TIMEOUT_ID);
    std::vector<uint8_t> response;
    this->on_response_(this->receive_(response) ? &response : nullptr);
  }
}

void PN5180::on_response_(const std::vector<uint8_t> *response) {
  const State state = this->state_;
  this->state_ = State::IDLE;

  if (state == State::INVENTORY) {
    // response: flags, DSFID, UID (8 bytes, LSB first)
    if (response == nullptr || response->size() < 10) {
      if (!this->uid_.empty()) {
        this->tag_removed_callback_.call(this->uid_string_());
        this->uid_.clear();
        this->publish_(nullptr);
      }
      return;
    }
    const std::vector<uint8_t> uid(response->begin() + 2, response->begin() + 10);
    if (uid == this->uid_ && this->tag_read_)
      return;
    if (uid != this->uid_) {
      this->uid_ = uid;
      this->tag_read_ = false;
      this->tag_callback_.call(this->uid_string_());
    }
    this->memory_.clear();
    this->read_next_();
    return;
  }

  if (state == State::WRITE) {
    // A write is confirmed after the tag programmed it, which can outlast the receive window;
    // the read after the last block shows whether it took
    if (response == nullptr)
      ESP_LOGW(TAG, "Writing block %u not confirmed", this->write_blocks_.front());
    this->write_blocks_.erase(this->write_blocks_.begin());
    this->write_next_();
    return;
  }

  // State::READ, response: flags, data
  if (response == nullptr || response->size() < 2 ||
      (!this->memory_.empty() && response->size() - 1 != size_t(this->read_count_) * this->block_size_)) {
    ESP_LOGW(TAG, "Reading block %zu failed", this->memory_.size() / this->block_size_);
    return;  // retried on the next update while the tag stays
  }
  if (this->memory_.empty())
    this->block_size_ = response->size() - 1;  // block 0 tells the block size
  this->memory_.insert(this->memory_.end(), response->begin() + 1, response->end());
  this->read_next_();
}

void PN5180::read_next_() {
  size_t size = 0;
  if (this->memory_.empty() || (this->memory_.size() < 8 && this->memory_.size() >= 4 && this->memory_[2] == 0)) {
    size = this->memory_.size() + 1;  // capability container: block 0, an 8 byte one also block 1
  } else if (this->memory_.size() >= 4) {
    size = std::min(t5t_memory_size(this->memory_), MAX_TAG_MEMORY);
  }
  const size_t block = this->memory_.size() / std::max<size_t>(this->block_size_, 1);
  if (this->memory_.size() >= size || block > 0xFF) {
    this->tag_read_ = true;
    this->publish_memory_();
    return;
  }
  const bool multiple = this->memory_.size() >= 4 && (this->memory_[3] & 0x01);  // MBREAD supported
  const size_t blocks = (size + this->block_size_ - 1) / this->block_size_;
  this->read_count_ = uint8_t(std::min<size_t>(multiple ? BLOCKS_PER_READ : 1, blocks - block));

  std::vector<uint8_t> frame = {ISO15693_FLAGS_ADDRESSED,
                                this->read_count_ == 1 ? ISO15693_READ_SINGLE_BLOCK : ISO15693_READ_MULTIPLE_BLOCKS};
  frame.insert(frame.end(), this->uid_.begin(), this->uid_.end());
  frame.push_back(block);
  if (this->read_count_ > 1)
    frame.push_back(this->read_count_ - 1);
  this->send_(State::READ, frame);
}

void PN5180::publish_memory_() {
  OpenPrintTag tag;
  if (parse_openprinttag(this->memory_, tag)) {
    ESP_LOGD(TAG, "OpenPrintTag %s: %zu main, %zu aux fields", this->uid_string_().c_str(), tag.main.size(),
             tag.aux.size());
    this->publish_(&tag);
    this->openprinttag_callback_.call(this->uid_string_(), format_hex(tag.payload.data(), tag.payload.size()));
  } else {
    ESP_LOGD(TAG, "Tag %s holds no OpenPrintTag record", this->uid_string_().c_str());
    this->publish_(nullptr);
  }
}

void PN5180::write_aux(const std::string &uid, const std::string &data_hex) {
  OpenPrintTag tag;
  if (uid.empty() || uid != this->uid_string_() || !this->tag_read_ || !parse_openprinttag(this->memory_, tag) ||
      tag.aux_size == 0) {
    ESP_LOGW(TAG, "No OpenPrintTag with an aux region and UID %s on the reader", uid.c_str());
    return;
  }
  std::vector<uint8_t> data(data_hex.size() / 2);
  const size_t start = tag.payload_offset + tag.aux_offset;
  if (data.empty() || !parse_hex(data_hex, data.data(), data.size()) || data.size() > tag.aux_size ||
      start + data.size() > this->memory_.size()) {
    ESP_LOGW(TAG, "Aux data invalid or larger than the aux region (%zu B)", tag.aux_size);
    return;
  }
  if (this->state_ != State::IDLE) {
    ESP_LOGW(TAG, "Reader busy, write not started");
    return;
  }
  // Only the blocks that differ; bytes after the new data stay (readers ignore them)
  this->written_ = this->memory_;
  std::copy(data.begin(), data.end(), this->written_.begin() + start);
  this->write_blocks_.clear();
  for (size_t block = start / this->block_size_; block * this->block_size_ < start + data.size(); block++) {
    const auto first = this->written_.begin() + block * this->block_size_;
    if (!std::equal(first, first + this->block_size_, this->memory_.begin() + block * this->block_size_))
      this->write_blocks_.push_back(block);
  }
  ESP_LOGD(TAG, "Writing %zu blocks of the aux region", this->write_blocks_.size());
  this->write_next_();
}

void PN5180::write_next_() {
  if (this->write_blocks_.empty()) {
    this->tag_read_ = false;  // read it again on the next update, that publishes what the tag holds
    return;
  }
  const uint8_t block = this->write_blocks_.front();
  std::vector<uint8_t> frame = {ISO15693_FLAGS_ADDRESSED, ISO15693_WRITE_SINGLE_BLOCK};
  frame.insert(frame.end(), this->uid_.begin(), this->uid_.end());
  frame.push_back(block);
  const auto first = this->written_.begin() + block * this->block_size_;
  frame.insert(frame.end(), first, first + this->block_size_);
  this->send_(State::WRITE, frame);
  if (this->state_ != State::WRITE) {  // the PN5180 did not take the frame
    ESP_LOGW(TAG, "Write aborted");
    this->write_blocks_.clear();
    this->tag_read_ = false;
  }
}

// UID with the 0xE0 manufacturer byte first, as OpenPrintTag (and the tag label) writes it
std::string PN5180::uid_string_() const {
  std::string out;
  char hex[4];
  for (auto it = this->uid_.rbegin(); it != this->uid_.rend(); ++it) {
    snprintf(hex, sizeof(hex), out.empty() ? "%02X" : "-%02X", *it);
    out += hex;
  }
  return out;
}

void PN5180::publish_(const OpenPrintTag *tag) {
  auto find = [tag](FieldSource source, uint32_t key) -> const OptValue * {
    if (tag == nullptr)
      return nullptr;
    const auto &region = source == SOURCE_AUX ? tag->aux : tag->main;
    auto it = region.find(key);
    return it == region.end() ? nullptr : &it->second;
  };

#ifdef USE_SENSOR
  for (auto &f : this->sensors_) {
    float value = NAN;
    if (f.source == SOURCE_REMAINING_WEIGHT) {
      const OptValue *full = find(SOURCE_MAIN, KEY_ACTUAL_WEIGHT);
      if (full == nullptr || !full->is_number)
        full = find(SOURCE_MAIN, KEY_NOMINAL_WEIGHT);
      const OptValue *consumed = find(SOURCE_AUX, KEY_AUX_CONSUMED_WEIGHT);
      if (full != nullptr && full->is_number)
        value = full->number - (consumed != nullptr && consumed->is_number ? consumed->number : 0);
    } else if (const OptValue *v = find(f.source, f.key); v != nullptr && v->is_number) {
      value = v->number;
    }
    f.sensor->publish_state(value);
  }
#endif

#ifdef USE_TEXT_SENSOR
  for (auto &f : this->text_sensors_) {
    std::string value;
    if (f.source == SOURCE_UID) {
      value = this->uid_string_();
    } else if (const OptValue *v = find(f.source, f.key); v != nullptr) {
      if (f.source == SOURCE_MAIN && f.key == KEY_MATERIAL_TYPE && v->is_number) {
        const auto i = size_t(v->number);
        value = i < std::size(MATERIAL_TYPES) ? MATERIAL_TYPES[i] : std::to_string(i);
      } else if (f.source == SOURCE_MAIN && f.key >= KEY_PRIMARY_COLOR && f.key <= KEY_SECONDARY_COLOR_4) {
        // RGBA, an opaque alpha is dropped so the value is a plain #rrggbb
        const size_t n = v->bytes.size() == 4 && uint8_t(v->bytes[3]) == 0xFF ? 3 : v->bytes.size();
        value = "#" + format_hex(reinterpret_cast<const uint8_t *>(v->bytes.data()), n);
      } else if (v->is_number) {
        char buf[24];
        snprintf(buf, sizeof(buf), "%.15g", v->number);
        value = buf;
      } else {
        value = v->bytes;
      }
    }
    if (!f.sensor->has_state() || f.sensor->state != value)
      f.sensor->publish_state(value);
  }
#endif
}

// ---- PN5180 host interface ----

bool PN5180::wait_busy_low_() {
  const uint32_t start = millis();
  while (this->busy_pin_->digital_read()) {
    if (millis() - start > BUSY_TIMEOUT_MS) {
      ESP_LOGW(TAG, "BUSY stuck high");
      return false;
    }
    delayMicroseconds(10);
  }
  return true;
}

bool PN5180::command_(const std::vector<uint8_t> &tx, uint8_t *rx, size_t rx_len) {
  if (!this->wait_busy_low_())
    return false;
  this->enable();
  this->write_array(tx);
  this->disable();
  // BUSY rises right after NSS and falls once the command is processed
  delayMicroseconds(5);
  if (!this->wait_busy_low_())
    return false;
  if (rx_len == 0)
    return true;
  std::fill(rx, rx + rx_len, 0xFF);
  this->enable();
  this->transfer_array(rx, rx_len);
  this->disable();
  delayMicroseconds(5);
  return this->wait_busy_low_();
}

bool PN5180::write_register_(uint8_t cmd, uint8_t reg, uint32_t value) {
  return this->command_({cmd, reg, uint8_t(value), uint8_t(value >> 8), uint8_t(value >> 16), uint8_t(value >> 24)});
}

bool PN5180::read_register_(uint8_t reg, uint32_t &value) {
  uint8_t rx[4];
  if (!this->command_({CMD_READ_REGISTER, reg}, rx, sizeof(rx)))
    return false;
  value = rx[0] | (rx[1] << 8) | (rx[2] << 16) | (uint32_t(rx[3]) << 24);
  return true;
}

bool PN5180::wait_irq_(uint32_t mask, uint32_t timeout_ms) {
  const uint32_t start = millis();
  uint32_t irq;
  do {
    if (!this->read_register_(REG_IRQ_STATUS, irq) || (irq & IRQ_GENERAL_ERROR))
      return false;
    if (irq & mask)
      return true;
    delayMicroseconds(200);
  } while (millis() - start < timeout_ms);
  return false;
}

bool PN5180::rf_on_() {
  this->write_register_(CMD_WRITE_REGISTER, REG_IRQ_CLEAR, IRQ_TX_RFON);
  if (!this->command_({CMD_RF_ON, 0x00}) || !this->wait_irq_(IRQ_TX_RFON, 500))
    return false;
  this->write_register_(CMD_WRITE_REGISTER, REG_IRQ_CLEAR, IRQ_TX_RFON);
  delay(10);  // let the tags power up
  return true;
}

void PN5180::send_(State state, const std::vector<uint8_t> &frame) {
  // Idle, then Transceive, which waits in WaitTransmit for SEND_DATA
  if (!this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, 0xFFFFFFF8) ||
      !this->write_register_(CMD_WRITE_REGISTER_OR_MASK, REG_SYSTEM_CONFIG, 0x00000003) ||
      !this->write_register_(CMD_WRITE_REGISTER, REG_IRQ_CLEAR, 0xFFFFFFFF))
    return;
  const uint32_t start = millis();
  uint32_t rf_status;
  do {
    if (!this->read_register_(REG_RF_STATUS, rf_status))
      return;
  } while (((rf_status >> 24) & 0x07) != TRANSCEIVE_STATE_WAIT_TRANSMIT && millis() - start < 10);

  std::vector<uint8_t> tx = {CMD_SEND_DATA, 0x00};  // 0 = all bits of the last byte valid
  tx.insert(tx.end(), frame.begin(), frame.end());
  if (!this->command_(tx))
    return;
  this->state_ = state;
  // No answer means no tag (or it left mid read)
  this->set_timeout(RX_TIMEOUT_ID, RX_TIMEOUT_MS, [this]() { this->on_response_(nullptr); });
  this->enable_loop();
}

bool PN5180::receive_(std::vector<uint8_t> &response) {
  uint32_t rx_status;
  if (!this->read_register_(REG_RX_STATUS, rx_status))
    return false;
  const size_t length = rx_status & RX_BYTES_MASK;
  if (length == 0 || length == RX_BYTES_MASK)
    return false;
  response.resize(length);
  if (!this->command_({CMD_READ_DATA, 0x00}, response.data(), length))
    return false;
  if (response[0] & ISO15693_ERROR_FLAG) {
    ESP_LOGV(TAG, "Tag error 0x%02X", length > 1 ? response[1] : 0);
    return false;
  }
  return true;
}

}  // namespace esphome::pn5180

#include "pn5180.h"

#include <algorithm>
#include <cstdio>

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
static constexpr uint8_t CMD_MFC_AUTHENTICATE = 0x0C;
static constexpr uint8_t CMD_LOAD_RF_CONFIG = 0x11;
static constexpr uint8_t CMD_RF_ON = 0x16;

// Registers
static constexpr uint8_t REG_SYSTEM_CONFIG = 0x00;
static constexpr uint8_t REG_IRQ_ENABLE = 0x01;
static constexpr uint8_t REG_IRQ_STATUS = 0x02;
static constexpr uint8_t REG_IRQ_CLEAR = 0x03;
static constexpr uint8_t REG_CRC_RX_CONFIG = 0x12;
static constexpr uint8_t REG_RX_STATUS = 0x13;
static constexpr uint8_t REG_CRC_TX_CONFIG = 0x19;
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
// ISO 14443A, 106 kbps
static constexpr uint8_t RF_CONFIG_ISO14443A_TX = 0x00;
static constexpr uint8_t RF_CONFIG_ISO14443A_RX = 0x80;
static constexpr uint8_t ISO14443A_WUPA = 0x52;  // wakes HALTed tags too, so a tag stays seen
static constexpr uint8_t ISO14443A_SEL_CL1 = 0x93;
static constexpr uint8_t ISO14443A_SEL_CL2 = 0x95;
static constexpr uint8_t ISO14443A_NVB_ANTICOLL = 0x20;
static constexpr uint8_t ISO14443A_NVB_SELECT = 0x70;
static constexpr uint8_t ISO14443A_CASCADE_TAG = 0x88;
static constexpr uint8_t ISO14443A_HLTA = 0x50;
static constexpr uint8_t MIFARE_AUTH_A = 0x60;
static constexpr uint8_t MIFARE_AUTH_B = 0x61;
static constexpr uint8_t MIFARE_READ = 0x30;
static constexpr uint8_t MIFARE_WRITE = 0xA0;
static constexpr uint8_t MIFARE_ACK = 0x0A;
static constexpr uint32_t SYSTEM_CONFIG_MFC_CRYPTO_ON = 1 << 6;

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
static constexpr uint8_t LOST_AFTER_MISSES = 3;  // polls without an answer before a tag counts as removed

// Memory size of a Type 5 tag from its capability container, 0 if it has none
static size_t type5_memory_size(const std::vector<uint8_t> &cc) {
  if (cc.size() < 4 || (cc[0] != 0xE1 && cc[0] != 0xE2))
    return 0;
  if (cc[2] != 0)
    return cc[2] * 8;
  return cc.size() < 8 ? 0 : ((cc[6] << 8) | cc[7]) * 8;  // 8 byte CC: MLEN in bytes 6-7
}


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
  ESP_LOGCONFIG(TAG, "PN5180 (ISO 15693, ISO 14443A UID):");
  LOG_PIN("  BUSY Pin: ", this->busy_pin_);
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  IRQ Pin: ", this->irq_pin_);
  LOG_UPDATE_INTERVAL(this);
  if (this->is_failed())
    ESP_LOGE(TAG, "  Setup failed");
}

// Each poll: an ISO 15693 inventory (e.g. OpenPrintTag, its memory read when new), then an
// ISO 14443A activation (NTAG, MIFARE, cards); one tag of each is tracked
void PN5180::update() {
  if (this->state_ == State::IDLE && this->set_type_a_(false))
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
    this->on_response_(this->receive_(response, this->state_ != State::TYPE_A) ? &response : nullptr);
  }
}

void PN5180::on_response_(const std::vector<uint8_t> *response) {
  const State state = this->state_;
  this->state_ = State::IDLE;

  if (state == State::INVENTORY) {
    // response: flags, DSFID, UID (8 bytes, LSB first)
    if (response == nullptr || response->size() < 10) {
      if (++this->nfcv_misses_ >= LOST_AFTER_MISSES)
        this->lost_nfcv_();
      this->poll_type_a_();
    } else {
      this->nfcv_misses_ = 0;
      this->found_nfcv_({response->begin() + 2, response->begin() + 10});
    }
    return;
  }

  if (state == State::TYPE_A) {
    this->on_type_a_(response);
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
    if (this->read_count_ > 1 && this->memory_.size() >= 4) {
      this->single_reads_ = true;  // some tags read fewer blocks at once
      this->read_next_();
      return;
    }
    ESP_LOGW(TAG, "Reading block %zu failed", this->memory_.size() / this->block_size_);
    this->poll_type_a_();  // the read is retried on the next update while the tag stays
    return;
  }
  if (this->memory_.empty())
    this->block_size_ = response->size() - 1;  // block 0 tells the block size
  this->memory_.insert(this->memory_.end(), response->begin() + 1, response->end());
  this->read_next_();
}

void PN5180::found_nfcv_(const std::vector<uint8_t> &uid) {
  if (uid != this->uid_) {
    this->lost_nfcv_();
    this->uid_ = uid;
    this->tag_read_ = false;
    this->announce_(this->uid_string_());
  }
  if (this->tag_read_) {
    this->poll_type_a_();
    return;
  }
  this->memory_.clear();
  this->single_reads_ = false;
  this->read_next_();  // ends with poll_type_a_()
}

void PN5180::found_nfca_(const std::vector<uint8_t> &uid, uint8_t sak) {
  if (uid != this->uid_a_) {
    this->lost_nfca_();
    this->uid_a_ = uid;
    this->announce_(this->uid_a_string_());
  }
  // The tag stays selected while the listeners read or write it
  this->mifare_ready_ = true;
  const std::string uid_string = this->uid_a_string_();
  for (auto *listener : this->listeners_)
    listener->on_iso14443a(uid_string, uid, sak);
  this->mifare_ready_ = false;
  // HALT it, the WUPA of the next poll wakes it again
  this->send_(State::IDLE, {ISO14443A_HLTA, 0x00}, 0, false);
  this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, ~SYSTEM_CONFIG_MFC_CRYPTO_ON);
}

void PN5180::lost_nfcv_() {
  if (this->uid_.empty())
    return;
  const std::string uid_string = this->uid_string_();
  this->uid_.clear();
  this->memory_.clear();
  this->removed_(uid_string);
}

void PN5180::lost_nfca_() {
  if (this->uid_a_.empty())
    return;
  const std::string uid_string = this->uid_a_string_();
  this->uid_a_.clear();
  this->removed_(uid_string);
}

void PN5180::announce_(const std::string &uid) {
  this->tag_callback_.call(uid);
  for (auto *listener : this->listeners_)
    listener->on_tag(uid);
#ifdef USE_TEXT_SENSOR
  if (this->uid_text_sensor_ != nullptr)
    this->uid_text_sensor_->publish_state(uid);
#endif
}

void PN5180::removed_(const std::string &uid) {
  this->tag_removed_callback_.call(uid);
  for (auto *listener : this->listeners_)
    listener->on_tag_removed(uid);
#ifdef USE_TEXT_SENSOR
  // the other tag, if one is still there
  if (this->uid_text_sensor_ != nullptr)
    this->uid_text_sensor_->publish_state(!this->uid_.empty() ? this->uid_string_() : this->uid_a_string_());
#endif
}

bool PN5180::set_type_a_(bool type_a) {
  if (this->rf_type_a_ == type_a)
    return true;
  // the transceiver has to be idle for a new RF configuration
  const bool ok = this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, 0xFFFFFFF8) && (type_a ? this->command_({CMD_LOAD_RF_CONFIG, RF_CONFIG_ISO14443A_TX, RF_CONFIG_ISO14443A_RX})
                         : this->command_({CMD_LOAD_RF_CONFIG, RF_CONFIG_ISO15693_TX, RF_CONFIG_ISO15693_RX}));
  if (ok)
    this->rf_type_a_ = type_a;
  return ok;
}

// The anticollision frames go without CRC, everything after them with it
bool PN5180::set_crc_(bool on) {
  if (on)
    return this->write_register_(CMD_WRITE_REGISTER_OR_MASK, REG_CRC_RX_CONFIG, 0x01) &&
           this->write_register_(CMD_WRITE_REGISTER_OR_MASK, REG_CRC_TX_CONFIG, 0x01);
  return this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_CRC_RX_CONFIG, 0xFFFFFFFE) &&
         this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_CRC_TX_CONFIG, 0xFFFFFFFE);
}

void PN5180::poll_type_a_() {
  this->type_a_step_ = TypeAStep::WUPA;
  if (this->set_type_a_(true) &&
      this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, ~SYSTEM_CONFIG_MFC_CRYPTO_ON) &&
      this->set_crc_(false))
    this->send_(State::TYPE_A, {ISO14443A_WUPA}, 7);  // a short frame of 7 bits
  if (this->state_ != State::TYPE_A)
    this->lost_nfca_();
}

// ponytail: single and double size UIDs (4, 7 bytes), triple size (10 bytes, rare) is not selected
void PN5180::on_type_a_(const std::vector<uint8_t> *response) {
  const auto step = this->type_a_step_;
  if (step == TypeAStep::WUPA) {
    if (response == nullptr || response->size() < 2) {  // ATQA
      if (++this->nfca_misses_ >= LOST_AFTER_MISSES)
        this->lost_nfca_();
      return;
    }
    this->nfca_misses_ = 0;
    this->type_a_step_ = TypeAStep::ANTICOLL_1;
    this->send_(State::TYPE_A, {ISO14443A_SEL_CL1, ISO14443A_NVB_ANTICOLL});
    return;
  }
  // A tag answered the WUPA: a failure later is retried on the next poll, not a removal
  if (response == nullptr)
    return;

  if (step == TypeAStep::ANTICOLL_1 || step == TypeAStep::ANTICOLL_2) {
    const auto &r = *response;
    if (r.size() != 5 || (r[0] ^ r[1] ^ r[2] ^ r[3]) != r[4])  // UID part + BCC
      return;
    const bool first = step == TypeAStep::ANTICOLL_1;
    if (first)
      std::copy(r.begin(), r.end(), this->cascade_1_);
    std::vector<uint8_t> select = {first ? ISO14443A_SEL_CL1 : ISO14443A_SEL_CL2, ISO14443A_NVB_SELECT};
    select.insert(select.end(), r.begin(), r.end());
    if (!first)
      std::copy(r.begin(), r.begin() + 4, this->cascade_2_);
    this->type_a_step_ = first ? TypeAStep::SELECT_1 : TypeAStep::SELECT_2;
    if (this->set_crc_(true))
      this->send_(State::TYPE_A, select);
    return;
  }

  // SAK
  std::vector<uint8_t> uid;
  if (step == TypeAStep::SELECT_1 && this->cascade_1_[0] == ISO14443A_CASCADE_TAG) {
    this->type_a_step_ = TypeAStep::ANTICOLL_2;
    if (this->set_crc_(false))
      this->send_(State::TYPE_A, {ISO14443A_SEL_CL2, ISO14443A_NVB_ANTICOLL});
    return;
  }
  if (step == TypeAStep::SELECT_1) {
    uid.assign(this->cascade_1_, this->cascade_1_ + 4);
  } else {
    uid.assign(this->cascade_1_ + 1, this->cascade_1_ + 4);  // CL1 starts with the cascade tag
    uid.insert(uid.end(), this->cascade_2_, this->cascade_2_ + 4);
  }
  ESP_LOGV(TAG, "ISO 14443A tag, SAK 0x%02X", (*response)[0]);
  this->found_nfca_(uid, (*response)[0]);
}

void PN5180::read_next_() {
  size_t size = 0;
  if (this->memory_.empty() || (this->memory_.size() < 8 && this->memory_.size() >= 4 && this->memory_[2] == 0)) {
    size = this->memory_.size() + 1;  // capability container: block 0, an 8 byte one also block 1
  } else if (this->memory_.size() >= 4) {
    size = std::min(type5_memory_size(this->memory_), MAX_TAG_MEMORY);
  }
  const size_t block = this->memory_.size() / std::max<size_t>(this->block_size_, 1);
  if (this->memory_.size() >= size || block > 0xFF) {
    this->tag_read_ = true;
    const std::string uid_string = this->uid_string_();
    for (auto *listener : this->listeners_)
      listener->on_type5_memory(uid_string, this->memory_);
    this->poll_type_a_();
    return;
  }
  const bool multiple = !this->single_reads_ && this->memory_.size() >= 4 && (this->memory_[3] & 0x01);  // MBREAD
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

bool PN5180::write(const std::string &uid, size_t offset, const std::vector<uint8_t> &data) {
  if (uid.empty() || uid != this->uid_string_() || !this->tag_read_ ||
      offset + data.size() > this->memory_.size()) {
    ESP_LOGW(TAG, "Write: no ISO 15693 tag %s on the reader, or past its memory", uid.c_str());
    return false;
  }
  if (this->state_ != State::IDLE) {
    ESP_LOGW(TAG, "Write: reader busy");
    return false;
  }
  // Only the blocks that differ
  this->written_ = this->memory_;
  std::copy(data.begin(), data.end(), this->written_.begin() + offset);
  this->write_blocks_.clear();
  for (size_t block = offset / this->block_size_; block * this->block_size_ < offset + data.size(); block++) {
    const auto first = this->written_.begin() + block * this->block_size_;
    if (!std::equal(first, first + this->block_size_, this->memory_.begin() + block * this->block_size_))
      this->write_blocks_.push_back(block);
  }
  ESP_LOGD(TAG, "Writing %zu blocks", this->write_blocks_.size());
  this->write_next_();
  return true;
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
static std::string uid_hex(const std::vector<uint8_t> &uid, bool reversed) {
  std::string out;
  char hex[4];
  for (size_t i = 0; i < uid.size(); i++) {
    snprintf(hex, sizeof(hex), out.empty() ? "%02X" : "-%02X", uid[reversed ? uid.size() - 1 - i : i]);
    out += hex;
  }
  return out;
}

std::string PN5180::uid_string_() const { return uid_hex(this->uid_, true); }
std::string PN5180::uid_a_string_() const { return uid_hex(this->uid_a_, false); }

// ---- MIFARE Classic, on the tag selected in found_nfca_ ----

bool PN5180::mifare_authenticate(uint8_t block, const uint8_t *key, bool key_b) {
  if (!this->mifare_ready_ || this->uid_a_.size() < 4)
    return false;
  // the PN5180 runs the Crypto1 handshake itself, with the last 4 UID bytes
  std::vector<uint8_t> cmd = {CMD_MFC_AUTHENTICATE};
  cmd.insert(cmd.end(), key, key + 6);
  cmd.push_back(key_b ? MIFARE_AUTH_B : MIFARE_AUTH_A);
  cmd.push_back(block);
  cmd.insert(cmd.end(), this->uid_a_.end() - 4, this->uid_a_.end());
  uint8_t status = 0xFF;
  if (this->command_(cmd, &status, 1) && status == 0x00)
    return true;
  this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, ~SYSTEM_CONFIG_MFC_CRYPTO_ON);
  this->mifare_ready_ = false;  // the tag is no longer selected
  return false;
}

bool PN5180::mifare_read(uint8_t block, uint8_t *data) {
  std::vector<uint8_t> response;
  if (!this->mifare_ready_ || !this->transceive_now_({MIFARE_READ, block}, response) || response.size() != 16)
    return false;
  std::copy(response.begin(), response.end(), data);
  return true;
}

bool PN5180::mifare_write(uint8_t block, const uint8_t *data) {
  if (!this->mifare_ready_)
    return false;
  // The tag answers each half with a 4 bit ACK, without CRC
  std::vector<uint8_t> response;
  bool ok = this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_CRC_RX_CONFIG, 0xFFFFFFFE) &&
            this->transceive_now_({MIFARE_WRITE, block}, response) && !response.empty() &&
            (response[0] & 0x0F) == MIFARE_ACK && this->transceive_now_({data, data + 16}, response) &&
            !response.empty() && (response[0] & 0x0F) == MIFARE_ACK;
  return this->write_register_(CMD_WRITE_REGISTER_OR_MASK, REG_CRC_RX_CONFIG, 0x01) && ok;
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

bool PN5180::transmit_(const std::vector<uint8_t> &frame, uint8_t valid_bits) {
  // Idle, then Transceive, which waits in WaitTransmit for SEND_DATA
  if (!this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, 0xFFFFFFF8) ||
      !this->write_register_(CMD_WRITE_REGISTER_OR_MASK, REG_SYSTEM_CONFIG, 0x00000003) ||
      !this->write_register_(CMD_WRITE_REGISTER, REG_IRQ_CLEAR, 0xFFFFFFFF))
    return false;
  const uint32_t start = millis();
  uint32_t rf_status;
  do {
    if (!this->read_register_(REG_RF_STATUS, rf_status))
      return false;
  } while (((rf_status >> 24) & 0x07) != TRANSCEIVE_STATE_WAIT_TRANSMIT && millis() - start < 10);

  std::vector<uint8_t> tx = {CMD_SEND_DATA, valid_bits};  // 0 = all bits of the last byte valid
  tx.insert(tx.end(), frame.begin(), frame.end());
  return this->command_(tx);
}

// ponytail: blocks a few ms per frame, fine for the handful of MIFARE frames per tag
bool PN5180::transceive_now_(const std::vector<uint8_t> &frame, std::vector<uint8_t> &response) {
  if (!this->transmit_(frame, 0))
    return false;
  const uint32_t start = millis();
  uint32_t irq;
  do {
    if (!this->read_register_(REG_IRQ_STATUS, irq) || (irq & IRQ_GENERAL_ERROR))
      return false;
    if (irq & IRQ_RX)
      return this->receive_(response, false);
    delayMicroseconds(100);
  } while (millis() - start < 20);
  return false;
}

void PN5180::send_(State state, const std::vector<uint8_t> &frame, uint8_t valid_bits, bool expect_response) {
  if (!this->transmit_(frame, valid_bits) || !expect_response)
    return;
  this->state_ = state;
  // No answer means no tag (or it left mid read)
  this->set_timeout(RX_TIMEOUT_ID, RX_TIMEOUT_MS, [this]() { this->on_response_(nullptr); });
  this->enable_loop();
}

bool PN5180::receive_(std::vector<uint8_t> &response, bool iso15693) {
  uint32_t rx_status;
  if (!this->read_register_(REG_RX_STATUS, rx_status))
    return false;
  const size_t length = rx_status & RX_BYTES_MASK;
  if (length == 0 || length == RX_BYTES_MASK)
    return false;
  response.resize(length);
  if (!this->command_({CMD_READ_DATA, 0x00}, response.data(), length))
    return false;
  if (iso15693 && (response[0] & ISO15693_ERROR_FLAG)) {
    ESP_LOGV(TAG, "Tag error 0x%02X", length > 1 ? response[1] : 0);
    return false;
  }
  return true;
}

}  // namespace esphome::pn5180

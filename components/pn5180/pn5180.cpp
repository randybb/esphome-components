#include "pn5180.h"

#include <algorithm>
#include <cstring>

#include "esphome/core/log.h"

namespace esphome::pn5180 {

ESPHOME_LOG_TAG(TAG, "pn5180");

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
static constexpr size_t MAX_BLOCK_SIZE = 32;              // ISO 15693 allows up to 256 bits
static constexpr size_t MAX_FRAME = 11 + MAX_BLOCK_SIZE;  // WRITE SINGLE BLOCK: flags, command, UID, block, data
static constexpr uint8_t LOST_AFTER_MISSES = 3;           // polls without an answer before a tag counts as removed

// Memory size of a Type 5 tag from its capability container, 0 if it has none
static size_t type5_memory_size(const std::vector<uint8_t> &cc) {
  if (cc.size() < 4 || (cc[0] != 0xE1 && cc[0] != 0xE2))
    return 0;
  if (cc[2] != 0)
    return cc[2] * 8;
  return cc.size() < 8 ? 0 : ((cc[6] << 8) | cc[7]) * 8;  // 8 byte CC: MLEN in bytes 6-7
}

// UID as "E0-04-...", the ISO 15693 one with the 0xE0 manufacturer byte first, as OpenPrintTag (and
// the tag label) writes it
static void uid_hex(char *out, const uint8_t *uid, size_t len, bool reversed) {
  for (size_t i = 0; i < len; i++) {
    const uint8_t b = uid[reversed ? len - 1 - i : i];
    if (i > 0)
      *out++ = '-';
    *out++ = format_hex_pretty_char(b >> 4);
    *out++ = format_hex_pretty_char(b & 0x0F);
  }
  *out = '\0';
}

void PN5180::setup() {
  this->memory_.reserve(MAX_TAG_MEMORY + MAX_BLOCK_SIZE);  // no allocation while reading tags
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
    this->irq_pin_->attach_interrupt(
        &PN5180::gpio_intr, this, this->irq_active_high_ ? gpio::INTERRUPT_RISING_EDGE : gpio::INTERRUPT_FALLING_EDGE);
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
    this->on_response_(false);
  } else if (irq & IRQ_RX) {
    this->cancel_timeout(RX_TIMEOUT_ID);
    this->on_response_(this->receive_(this->state_ != State::TYPE_A));
  }
}

// The answer is in rx_ when `ok`
void PN5180::on_response_(bool ok) {
  const State state = this->state_;
  this->state_ = State::IDLE;

  if (state == State::INVENTORY) {
    // response: flags, DSFID, UID (8 bytes, LSB first)
    if (!ok || this->rx_len_ < 10) {
      if (++this->nfcv_misses_ >= LOST_AFTER_MISSES)
        this->lost_nfcv_();
      this->poll_type_a_();
    } else {
      this->nfcv_misses_ = 0;
      this->found_nfcv_(this->rx_.data() + 2);
    }
    return;
  }

  if (state == State::TYPE_A) {
    this->on_type_a_(ok);
    return;
  }

  if (state == State::WRITE) {
    // A write is confirmed after the tag programmed it, which can outlast the receive window;
    // the read after the last block shows whether it took
    if (!ok)
      ESP_LOGW(TAG, "Writing block %u not confirmed", this->write_block_);
    this->write_blocks_.reset(this->write_block_);
    this->write_next_();
    return;
  }

  // State::READ, response: flags, data
  const size_t data_len = this->rx_len_ - 1;
  if (!ok || this->rx_len_ < 2 ||
      (this->memory_.empty() ? data_len > MAX_BLOCK_SIZE : data_len != size_t(this->read_count_) * this->block_size_)) {
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
    this->block_size_ = data_len;  // block 0 tells the block size
  this->memory_.insert(this->memory_.end(), this->rx_.begin() + 1, this->rx_.begin() + this->rx_len_);
  this->read_next_();
}

void PN5180::found_nfcv_(const uint8_t *uid) {
  if (!this->has_uid_ || !std::equal(uid, uid + this->uid_.size(), this->uid_.begin())) {
    this->lost_nfcv_();
    std::copy(uid, uid + this->uid_.size(), this->uid_.begin());
    this->has_uid_ = true;
    this->tag_read_ = false;
    uid_hex(this->uid_str_, this->uid_.data(), this->uid_.size(), true);
    this->announce_(this->uid_str_);
  }
  if (this->tag_read_) {
    this->poll_type_a_();
    return;
  }
  this->memory_.clear();
  this->single_reads_ = false;
  this->read_next_();  // ends with poll_type_a_()
}

void PN5180::found_nfca_(const uint8_t *uid, size_t len, uint8_t sak) {
  if (len != this->uid_a_len_ || !std::equal(uid, uid + len, this->uid_a_.begin())) {
    this->lost_nfca_();
    std::copy(uid, uid + len, this->uid_a_.begin());
    this->uid_a_len_ = len;
    uid_hex(this->uid_a_str_, uid, len, false);
    this->announce_(this->uid_a_str_);
  }
  // The tag stays selected while the listeners read or write it
  this->mifare_ready_ = true;
  for (auto *listener : this->listeners_)
    listener->on_iso14443a(this->uid_a_str_, this->uid_a_.data(), this->uid_a_len_, sak);
  this->mifare_ready_ = false;
  // HALT it, the WUPA of the next poll wakes it again
  this->send_(State::IDLE, {ISO14443A_HLTA, 0x00}, 0, false);
  this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, ~SYSTEM_CONFIG_MFC_CRYPTO_ON);
}

void PN5180::lost_nfcv_() {
  if (!this->has_uid_)
    return;
  this->has_uid_ = false;
  this->memory_.clear();
  this->removed_(this->uid_str_);
  this->uid_str_[0] = '\0';
}

void PN5180::lost_nfca_() {
  if (this->uid_a_len_ == 0)
    return;
  this->uid_a_len_ = 0;
  this->removed_(this->uid_a_str_);
  this->uid_a_str_[0] = '\0';
}

void PN5180::announce_(const char *uid) {
  this->tag_callback_.call(uid);
  for (auto *listener : this->listeners_)
    listener->on_tag(uid);
#ifdef USE_TEXT_SENSOR
  if (this->uid_text_sensor_ != nullptr)
    this->uid_text_sensor_->publish_state(uid);
#endif
}

void PN5180::removed_(const char *uid) {
  this->tag_removed_callback_.call(uid);
  for (auto *listener : this->listeners_)
    listener->on_tag_removed(uid);
#ifdef USE_TEXT_SENSOR
  // the other tag, if one is still there
  if (this->uid_text_sensor_ != nullptr)
    this->uid_text_sensor_->publish_state(this->has_uid_          ? this->uid_str_
                                          : this->uid_a_len_ != 0 ? this->uid_a_str_
                                                                  : "");
#endif
}

bool PN5180::set_type_a_(bool type_a) {
  if (this->rf_type_a_ == type_a)
    return true;
  // the transceiver has to be idle for a new RF configuration
  const bool ok = this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, 0xFFFFFFF8) &&
                  (type_a ? this->command_({CMD_LOAD_RF_CONFIG, RF_CONFIG_ISO14443A_TX, RF_CONFIG_ISO14443A_RX})
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
void PN5180::on_type_a_(bool ok) {
  const auto step = this->type_a_step_;
  if (step == TypeAStep::WUPA) {
    if (!ok || this->rx_len_ < 2) {  // ATQA
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
  if (!ok)
    return;

  const auto &r = this->rx_;
  if (step == TypeAStep::ANTICOLL_1 || step == TypeAStep::ANTICOLL_2) {
    if (this->rx_len_ != 5 || (r[0] ^ r[1] ^ r[2] ^ r[3]) != r[4])  // UID part + BCC
      return;
    const bool first = step == TypeAStep::ANTICOLL_1;
    if (first) {
      std::copy(r.begin(), r.begin() + 5, this->cascade_1_);
    } else {
      std::copy(r.begin(), r.begin() + 4, this->cascade_2_);
    }
    const uint8_t select[] = {
        first ? ISO14443A_SEL_CL1 : ISO14443A_SEL_CL2, ISO14443A_NVB_SELECT, r[0], r[1], r[2], r[3], r[4]};
    this->type_a_step_ = first ? TypeAStep::SELECT_1 : TypeAStep::SELECT_2;
    if (this->set_crc_(true))
      this->send_(State::TYPE_A, select, sizeof(select));
    return;
  }

  // SAK
  if (step == TypeAStep::SELECT_1 && this->cascade_1_[0] == ISO14443A_CASCADE_TAG) {
    this->type_a_step_ = TypeAStep::ANTICOLL_2;
    if (this->set_crc_(false))
      this->send_(State::TYPE_A, {ISO14443A_SEL_CL2, ISO14443A_NVB_ANTICOLL});
    return;
  }
  uint8_t uid[7];
  size_t len;
  if (step == TypeAStep::SELECT_1) {
    std::copy(this->cascade_1_, this->cascade_1_ + 4, uid);
    len = 4;
  } else {
    std::copy(this->cascade_1_ + 1, this->cascade_1_ + 4, uid);  // CL1 starts with the cascade tag
    std::copy(this->cascade_2_, this->cascade_2_ + 4, uid + 3);
    len = 7;
  }
  const uint8_t sak = r[0];
  ESP_LOGV(TAG, "ISO 14443A tag, SAK 0x%02X", sak);
  this->found_nfca_(uid, len, sak);
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
    for (auto *listener : this->listeners_)
      listener->on_type5_memory(this->uid_str_, this->memory_);
    this->poll_type_a_();
    return;
  }
  const bool multiple = !this->single_reads_ && this->memory_.size() >= 4 && (this->memory_[3] & 0x01);  // MBREAD
  const size_t blocks = (size + this->block_size_ - 1) / this->block_size_;
  this->read_count_ = uint8_t(std::min<size_t>(multiple ? BLOCKS_PER_READ : 1, blocks - block));

  uint8_t frame[12] = {ISO15693_FLAGS_ADDRESSED,
                       this->read_count_ == 1 ? ISO15693_READ_SINGLE_BLOCK : ISO15693_READ_MULTIPLE_BLOCKS};
  size_t len = 2;
  std::copy(this->uid_.begin(), this->uid_.end(), frame + len);
  len += this->uid_.size();
  frame[len++] = block;
  if (this->read_count_ > 1)
    frame[len++] = this->read_count_ - 1;
  this->send_(State::READ, frame, len);
}

bool PN5180::write(const char *uid, size_t offset, const uint8_t *data, size_t len) {
  if (uid == nullptr || !this->has_uid_ || strcmp(uid, this->uid_str_) != 0 || !this->tag_read_ ||
      offset + len > this->memory_.size()) {
    ESP_LOGW(TAG, "Write: no ISO 15693 tag %s on the reader, or past its memory", uid != nullptr ? uid : "");
    return false;
  }
  if (this->state_ != State::IDLE) {
    ESP_LOGW(TAG, "Write: reader busy");
    return false;
  }
  // Only the blocks that differ; memory_ holds the new content until the tag is read again
  this->write_blocks_.reset();
  for (size_t i = 0; i < len; i++) {
    if (data[i] != this->memory_[offset + i])
      this->write_blocks_.set((offset + i) / this->block_size_);
  }
  std::copy(data, data + len, this->memory_.begin() + offset);
  ESP_LOGD(TAG, "Writing %zu blocks", this->write_blocks_.count());
  this->write_next_();
  return true;
}

void PN5180::write_next_() {
  size_t block = 0;
  while (block < this->write_blocks_.size() && !this->write_blocks_.test(block))
    block++;
  if (block == this->write_blocks_.size()) {
    this->tag_read_ = false;  // read it again on the next update, that publishes what the tag holds
    return;
  }
  this->write_block_ = block;
  uint8_t frame[MAX_FRAME] = {ISO15693_FLAGS_ADDRESSED, ISO15693_WRITE_SINGLE_BLOCK};
  size_t len = 2;
  std::copy(this->uid_.begin(), this->uid_.end(), frame + len);
  len += this->uid_.size();
  frame[len++] = block;
  const auto first = this->memory_.begin() + block * this->block_size_;
  std::copy(first, first + this->block_size_, frame + len);
  len += this->block_size_;
  this->send_(State::WRITE, frame, len);
  if (this->state_ != State::WRITE) {  // the PN5180 did not take the frame
    ESP_LOGW(TAG, "Write aborted");
    this->write_blocks_.reset();
    this->tag_read_ = false;
  }
}

// ---- MIFARE Classic, on the tag selected in found_nfca_ ----

bool PN5180::mifare_authenticate(uint8_t block, const uint8_t *key, bool key_b) {
  if (!this->mifare_ready_ || this->uid_a_len_ < 4)
    return false;
  // the PN5180 runs the Crypto1 handshake itself, with the last 4 UID bytes
  uint8_t cmd[13] = {CMD_MFC_AUTHENTICATE};
  std::copy(key, key + 6, cmd + 1);
  cmd[7] = key_b ? MIFARE_AUTH_B : MIFARE_AUTH_A;
  cmd[8] = block;
  std::copy(this->uid_a_.begin() + this->uid_a_len_ - 4, this->uid_a_.begin() + this->uid_a_len_, cmd + 9);
  uint8_t status = 0xFF;
  if (this->command_(cmd, sizeof(cmd), &status, 1) && status == 0x00)
    return true;
  this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_SYSTEM_CONFIG, ~SYSTEM_CONFIG_MFC_CRYPTO_ON);
  this->mifare_ready_ = false;  // the tag is no longer selected
  return false;
}

bool PN5180::mifare_read(uint8_t block, uint8_t *data) {
  if (!this->mifare_ready_ || !this->transceive_now_({MIFARE_READ, block}) || this->rx_len_ != 16)
    return false;
  std::copy(this->rx_.begin(), this->rx_.begin() + 16, data);
  return true;
}

bool PN5180::mifare_write(uint8_t block, const uint8_t *data) {
  if (!this->mifare_ready_)
    return false;
  // The tag answers each half with a 4 bit ACK, without CRC
  bool ok = this->write_register_(CMD_WRITE_REGISTER_AND_MASK, REG_CRC_RX_CONFIG, 0xFFFFFFFE) &&
            this->transceive_now_({MIFARE_WRITE, block}) && this->rx_len_ > 0 && (this->rx_[0] & 0x0F) == MIFARE_ACK &&
            this->transceive_now_(data, 16) && this->rx_len_ > 0 && (this->rx_[0] & 0x0F) == MIFARE_ACK;
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

bool PN5180::command_(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len) {
  if (!this->wait_busy_low_())
    return false;
  this->enable();
  this->write_array(tx, tx_len);
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

// Only in setup(), the tags have powered up by the first poll
bool PN5180::rf_on_() {
  this->write_register_(CMD_WRITE_REGISTER, REG_IRQ_CLEAR, IRQ_TX_RFON);
  if (!this->command_({CMD_RF_ON, 0x00}) || !this->wait_irq_(IRQ_TX_RFON, 500))
    return false;
  return this->write_register_(CMD_WRITE_REGISTER, REG_IRQ_CLEAR, IRQ_TX_RFON);
}

bool PN5180::transmit_(const uint8_t *frame, size_t len, uint8_t valid_bits) {
  if (len > MAX_FRAME)
    return false;
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

  uint8_t tx[2 + MAX_FRAME] = {CMD_SEND_DATA, valid_bits};  // 0 = all bits of the last byte valid
  std::copy(frame, frame + len, tx + 2);
  return this->command_(tx, 2 + len);
}

// Waits for the answer, for the MIFARE frames inside on_iso14443a: a few ms each, 20 ms without one
bool PN5180::transceive_now_(const uint8_t *frame, size_t len) {
  if (!this->transmit_(frame, len, 0))
    return false;
  const uint32_t start = millis();
  uint32_t irq;
  do {
    if (!this->read_register_(REG_IRQ_STATUS, irq) || (irq & IRQ_GENERAL_ERROR))
      return false;
    if (irq & IRQ_RX)
      return this->receive_(false);
    delayMicroseconds(100);
  } while (millis() - start < 20);
  return false;
}

void PN5180::send_(State state, const uint8_t *frame, size_t len, uint8_t valid_bits, bool expect_response) {
  if (!this->transmit_(frame, len, valid_bits) || !expect_response)
    return;
  this->state_ = state;
  // No answer means no tag (or it left mid read)
  this->set_timeout(RX_TIMEOUT_ID, RX_TIMEOUT_MS, [this]() { this->on_response_(false); });
  this->enable_loop();
}

// Reads the answer into rx_
bool PN5180::receive_(bool iso15693) {
  this->rx_len_ = 0;
  uint32_t rx_status;
  if (!this->read_register_(REG_RX_STATUS, rx_status))
    return false;
  const size_t length = rx_status & RX_BYTES_MASK;
  if (length == 0 || length > this->rx_.size())
    return false;
  if (!this->command_({CMD_READ_DATA, 0x00}, this->rx_.data(), length))
    return false;
  this->rx_len_ = length;
  if (iso15693 && (this->rx_[0] & ISO15693_ERROR_FLAG)) {
    ESP_LOGV(TAG, "Tag error 0x%02X", length > 1 ? this->rx_[1] : 0);
    return false;
  }
  return true;
}

}  // namespace esphome::pn5180

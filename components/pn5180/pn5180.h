#pragma once

#include <array>
#include <bitset>
#include <initializer_list>
#include <string>
#include <vector>

#include "esphome/components/spi/spi.h"
#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::pn5180 {

// A component decoding the tags the PN5180 reads (e.g. openprinttag), registered with add_listener()
class TagListener {
 public:
  // A new tag is in the field (any kind), `uid` as in on_tag
  virtual void on_tag(const char *uid) {}
  // The memory of an ISO 15693 (NFC Forum Type 5) tag was read, from block 0 (CC) up to the size
  // the CC tells; again after every write
  virtual void on_type5_memory(const char *uid, const std::vector<uint8_t> &memory) {}
  // An ISO 14443A tag is selected, on every poll while it stays (`uid_bytes` MSB first, `sak` tells
  // the kind, 0x08 MIFARE Classic 1K); mifare_authenticate/read/write work only inside this call
  virtual void on_iso14443a(const char *uid, const uint8_t *uid_bytes, size_t uid_len, uint8_t sak) {}
  // The tag left the field
  virtual void on_tag_removed(const char *uid) {}
};

class PN5180 final : public PollingComponent,
                     public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                           spi::DATA_RATE_4MHZ> {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;

  void set_busy_pin(GPIOPin *pin) { this->busy_pin_ = pin; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_irq_pin(InternalGPIOPin *pin) { this->irq_pin_ = pin; }
#ifdef USE_TEXT_SENSOR
  void set_uid_text_sensor(text_sensor::TextSensor *sensor) { this->uid_text_sensor_ = sensor; }
#endif

  void add_listener(TagListener *listener) { this->listeners_.push_back(listener); }
  template<typename F> void add_on_tag_callback(F &&callback) { this->tag_callback_.add(std::forward<F>(callback)); }
  template<typename F> void add_on_tag_removed_callback(F &&callback) {
    this->tag_removed_callback_.add(std::forward<F>(callback));
  }

  // Writes `data` at byte `offset` of the memory of the ISO 15693 tag with UID `uid`, as passed to
  // on_type5_memory. Only the blocks that change are written, then the tag is read again, so the
  // listeners get what really is on it. False when the tag is not there or the reader is busy.
  bool write(const char *uid, size_t offset, const uint8_t *data, size_t len);

  // MIFARE Classic access to the selected ISO 14443A tag, only inside TagListener::on_iso14443a.
  // A failed authentication drops the tag's selection: nothing works after it in that call.
  bool mifare_authenticate(uint8_t block, const uint8_t *key, bool key_b = false);
  bool mifare_read(uint8_t block, uint8_t *data);  // 16 bytes
  bool mifare_write(uint8_t block, const uint8_t *data);

 protected:
  bool command_(const uint8_t *tx, size_t tx_len, uint8_t *rx = nullptr, size_t rx_len = 0);
  bool command_(std::initializer_list<uint8_t> tx, uint8_t *rx = nullptr, size_t rx_len = 0) {
    return this->command_(tx.begin(), tx.size(), rx, rx_len);
  }
  bool wait_busy_low_();
  bool write_register_(uint8_t cmd, uint8_t reg, uint32_t value);
  bool read_register_(uint8_t reg, uint32_t &value);
  bool wait_irq_(uint32_t mask, uint32_t timeout_ms);
  bool rf_on_();

  enum class State : uint8_t { IDLE, INVENTORY, TYPE_A, READ, WRITE };
  // ISO 14443A activation steps, in State::TYPE_A
  enum class TypeAStep : uint8_t { WUPA, ANTICOLL_1, SELECT_1, ANTICOLL_2, SELECT_2 };
  void send_(State state, const uint8_t *frame, size_t len, uint8_t valid_bits = 0, bool expect_response = true);
  void send_(State state, std::initializer_list<uint8_t> frame, uint8_t valid_bits = 0, bool expect_response = true) {
    this->send_(state, frame.begin(), frame.size(), valid_bits, expect_response);
  }
  bool set_type_a_(bool type_a);
  bool set_crc_(bool on);
  void poll_type_a_();
  void on_type_a_(bool ok);
  void found_nfcv_(const uint8_t *uid);
  void found_nfca_(const uint8_t *uid, size_t len, uint8_t sak);
  void lost_nfcv_();
  void lost_nfca_();
  void announce_(const char *uid);
  void removed_(const char *uid);
  bool transmit_(const uint8_t *frame, size_t len, uint8_t valid_bits);
  bool transceive_now_(const uint8_t *frame, size_t len);
  bool transceive_now_(std::initializer_list<uint8_t> frame) {
    return this->transceive_now_(frame.begin(), frame.size());
  }
  bool receive_(bool iso15693 = true);
  void on_response_(bool ok);
  void read_next_();
  void write_next_();
  static void gpio_intr(PN5180 *arg);

  GPIOPin *busy_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  InternalGPIOPin *irq_pin_{nullptr};
  bool irq_active_high_{true};
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *uid_text_sensor_{nullptr};
#endif

  State state_{State::IDLE};
  // The last answer of a tag, filled by receive_()
  std::array<uint8_t, 260> rx_{};
  size_t rx_len_{0};
  std::vector<uint8_t> memory_;    // tag memory read so far, kept for writes; reserved in setup()
  std::bitset<256> write_blocks_;  // blocks still to write
  uint8_t write_block_{0};         // the one being written
  uint8_t read_count_{0};          // blocks in the pending read
  bool single_reads_{false};       // the tag failed a multiple block read

  // One tag of each protocol is tracked, a spool carries an OpenPrintTag and Creality tags
  std::array<uint8_t, 8> uid_{};  // ISO 15693 tag, LSB first as sent over the air
  bool has_uid_{false};
  std::array<uint8_t, 7> uid_a_{};  // ISO 14443A tag, MSB first
  uint8_t uid_a_len_{0};
  char uid_str_[24]{};  // as passed to the listeners, "E0-04-..."
  char uid_a_str_[21]{};
  bool mifare_ready_{false};  // inside on_iso14443a
  // Polls without an answer: two tags close to the antenna detune it, single frames get lost
  uint8_t nfcv_misses_{0};
  uint8_t nfca_misses_{0};
  bool rf_type_a_{false};  // RF configuration loaded: ISO 14443A, else ISO 15693
  TypeAStep type_a_step_{TypeAStep::WUPA};
  uint8_t cascade_1_[5]{};  // UID CL1 + BCC
  uint8_t cascade_2_[4]{};  // UID CL2
  bool tag_read_{false};    // the ISO 15693 memory
  uint8_t block_size_{4};

  std::vector<TagListener *> listeners_;
  CallbackManager<void(std::string)> tag_callback_;
  CallbackManager<void(std::string)> tag_removed_callback_;
};

}  // namespace esphome::pn5180

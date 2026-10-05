#pragma once

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
  virtual void on_tag(const std::string &uid) {}
  // The memory of an ISO 15693 (NFC Forum Type 5) tag was read, from block 0 (CC) up to the size
  // the CC tells; again after every write
  virtual void on_type5_memory(const std::string &uid, const std::vector<uint8_t> &memory) {}
  // An ISO 14443A tag is selected, on every poll while it stays (`uid_bytes` MSB first, `sak` tells
  // the kind, 0x08 MIFARE Classic 1K); mifare_authenticate/read/write work only inside this call
  virtual void on_iso14443a(const std::string &uid, const std::vector<uint8_t> &uid_bytes, uint8_t sak) {}
  // The tag left the field
  virtual void on_tag_removed(const std::string &uid) {}
};

class PN5180 : public PollingComponent,
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
  bool write(const std::string &uid, size_t offset, const std::vector<uint8_t> &data);

  // MIFARE Classic access to the selected ISO 14443A tag, only inside TagListener::on_iso14443a.
  // A failed authentication drops the tag's selection: nothing works after it in that call.
  bool mifare_authenticate(uint8_t block, const uint8_t *key, bool key_b = false);
  bool mifare_read(uint8_t block, uint8_t *data);  // 16 bytes
  bool mifare_write(uint8_t block, const uint8_t *data);

 protected:
  bool command_(const std::vector<uint8_t> &tx, uint8_t *rx = nullptr, size_t rx_len = 0);
  bool wait_busy_low_();
  bool write_register_(uint8_t cmd, uint8_t reg, uint32_t value);
  bool read_register_(uint8_t reg, uint32_t &value);
  bool wait_irq_(uint32_t mask, uint32_t timeout_ms);
  bool rf_on_();

  enum class State : uint8_t { IDLE, INVENTORY, TYPE_A, READ, WRITE };
  // ISO 14443A activation steps, in State::TYPE_A
  enum class TypeAStep : uint8_t { WUPA, ANTICOLL_1, SELECT_1, ANTICOLL_2, SELECT_2 };
  void send_(State state, const std::vector<uint8_t> &frame, uint8_t valid_bits = 0, bool expect_response = true);
  bool set_type_a_(bool type_a);
  bool set_crc_(bool on);
  void poll_type_a_();
  void on_type_a_(const std::vector<uint8_t> *response);
  void found_nfcv_(const std::vector<uint8_t> &uid);
  void found_nfca_(const std::vector<uint8_t> &uid, uint8_t sak);
  void lost_nfcv_();
  void lost_nfca_();
  void announce_(const std::string &uid);
  void removed_(const std::string &uid);
  bool transmit_(const std::vector<uint8_t> &frame, uint8_t valid_bits);
  bool transceive_now_(const std::vector<uint8_t> &frame, std::vector<uint8_t> &response);
  bool receive_(std::vector<uint8_t> &response, bool iso15693 = true);
  void on_response_(const std::vector<uint8_t> *response);
  void read_next_();
  void write_next_();
  static void gpio_intr(PN5180 *arg);
  std::string uid_string_() const;    // of the ISO 15693 tag
  std::string uid_a_string_() const;  // of the ISO 14443A tag

  GPIOPin *busy_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  InternalGPIOPin *irq_pin_{nullptr};
  bool irq_active_high_{true};
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *uid_text_sensor_{nullptr};
#endif

  State state_{State::IDLE};
  std::vector<uint8_t> memory_;   // tag memory read so far, kept for writes
  std::vector<uint8_t> written_;  // tag memory after the pending writes
  std::vector<uint8_t> write_blocks_;
  uint8_t read_count_{0};     // blocks in the pending read
  bool single_reads_{false};  // the tag failed a multiple block read

  // One tag of each protocol is tracked, a spool carries an OpenPrintTag and Creality tags
  std::vector<uint8_t> uid_;    // ISO 15693 tag, LSB first as sent over the air
  std::vector<uint8_t> uid_a_;  // ISO 14443A tag, MSB first
  bool mifare_ready_{false};    // inside on_iso14443a
  // Polls without an answer: two tags close to the antenna detune it, single frames get lost
  uint8_t nfcv_misses_{0};
  uint8_t nfca_misses_{0};
  bool rf_type_a_{false};       // RF configuration loaded: ISO 14443A, else ISO 15693
  TypeAStep type_a_step_{TypeAStep::WUPA};
  uint8_t cascade_1_[5]{};  // UID CL1 + BCC
  uint8_t cascade_2_[4]{};  // UID CL2
  bool tag_read_{false};  // the ISO 15693 memory
  uint8_t block_size_{4};

  std::vector<TagListener *> listeners_;
  CallbackManager<void(std::string)> tag_callback_;
  CallbackManager<void(std::string)> tag_removed_callback_;
};

}  // namespace esphome::pn5180

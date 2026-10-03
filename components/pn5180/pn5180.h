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
  void found_tag_(const std::vector<uint8_t> &uid, bool type_a);
  void no_tag_();
  bool receive_(std::vector<uint8_t> &response);
  void on_response_(const std::vector<uint8_t> *response);
  void read_next_();
  void write_next_();
  static void gpio_intr(PN5180 *arg);
  std::string uid_string_() const;

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

  std::vector<uint8_t> uid_;  // as sent over the air: ISO 15693 LSB first, ISO 14443A MSB first
  bool uid_type_a_{false};
  bool rf_type_a_{false};  // RF configuration loaded: ISO 14443A, else ISO 15693
  TypeAStep type_a_step_{TypeAStep::WUPA};
  uint8_t cascade_1_[5]{};  // UID CL1 + BCC
  uint8_t cascade_2_[4]{};  // UID CL2
  bool tag_read_{false};
  uint8_t block_size_{4};

  std::vector<TagListener *> listeners_;
  CallbackManager<void(std::string)> tag_callback_;
  CallbackManager<void(std::string)> tag_removed_callback_;
};

}  // namespace esphome::pn5180

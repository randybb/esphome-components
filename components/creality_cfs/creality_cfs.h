#pragma once

#include <string>
#include <vector>

#include "esphome/components/pn5180/pn5180.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::creality_cfs {

// A field of the 48 character record: [offset, offset + length)
struct Field {
  uint8_t offset;
  uint8_t length;
};

// Reads and writes Creality CFS filament tags (MIFARE Classic 1K) on a reader
class CrealityCfsComponent : public Component, public pn5180::TagListener {
 public:
  void dump_config() override;

  void set_reader(pn5180::PN5180 *reader) {
    this->reader_ = reader;
    reader->add_listener(this);
  }
  // uid, the record (sector 1 decrypted, then sector 2) as ASCII; empty: the tag went away
  template<typename F> void add_on_creality_callback(F &&callback) {
    this->creality_callback_.add(std::forward<F>(callback));
  }
#ifdef USE_SENSOR
  void set_length_sensor(sensor::Sensor *sensor) { this->length_sensor_ = sensor; }
#endif
#ifdef USE_TEXT_SENSOR
  void add_text_sensor(text_sensor::TextSensor *sensor, uint8_t offset, uint8_t length, bool color) {
    this->text_sensors_.push_back({sensor, {offset, length}, color});
  }
#endif

  // Writes `data` (48 characters of the record for sector 1, optionally 48 more for sector 2) to
  // the tag with UID `uid` on its next poll; a blank tag gets its keys derived from the UID
  void write(const std::string &uid, const std::string &data);

  void on_iso14443a(const std::string &uid, const std::vector<uint8_t> &uid_bytes, uint8_t sak) override;
  void on_tag_removed(const std::string &uid) override;

 protected:
  bool read_(bool blank, std::string &record);
  bool write_(bool blank);
  void publish_(const std::string &record);

  pn5180::PN5180 *reader_{nullptr};
  std::string uid_;           // of the Creality tag on the reader
  bool published_{false};
  bool read_done_{false};
  bool default_key_{false};   // the next authentication uses the factory key (a blank tag)
  uint8_t key_[6]{};          // derived from the UID
  std::string pending_uid_;   // a write waiting for its tag
  std::string pending_data_;
  CallbackManager<void(std::string, std::string)> creality_callback_;

#ifdef USE_SENSOR
  sensor::Sensor *length_sensor_{nullptr};
#endif
#ifdef USE_TEXT_SENSOR
  struct TextSensorField {
    text_sensor::TextSensor *sensor;
    Field field;
    bool color;
  };
  std::vector<TextSensorField> text_sensors_;
#endif
};

}  // namespace esphome::creality_cfs

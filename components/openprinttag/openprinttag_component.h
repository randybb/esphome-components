#pragma once

#include <string>
#include <vector>

#include "esphome/components/pn5180/pn5180.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "openprinttag.h"

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome::openprinttag {

// Where a sensor takes its value from
enum FieldSource : uint8_t {
  SOURCE_MAIN = 0,
  SOURCE_AUX = 1,
  SOURCE_REMAINING_WEIGHT = 3,  // numeric only: (actual or nominal) full weight - consumed weight
};

// Decodes the OpenPrintTag tags a reader reads, to sensors and the on_openprinttag trigger
class OpenPrintTagComponent : public Component, public pn5180::TagListener {
 public:
  void dump_config() override;

  void set_reader(pn5180::PN5180 *reader) {
    this->reader_ = reader;
    reader->add_listener(this);
  }
  // uid, the NDEF record payload as hex; empty payload: the tag went away
  template<typename F> void add_on_openprinttag_callback(F &&callback) {
    this->openprinttag_callback_.add(std::forward<F>(callback));
  }
#ifdef USE_SENSOR
  void add_sensor(sensor::Sensor *sensor, FieldSource source, uint32_t key) {
    this->sensors_.push_back({sensor, source, key});
  }
#endif
#ifdef USE_TEXT_SENSOR
  void add_text_sensor(text_sensor::TextSensor *sensor, FieldSource source, uint32_t key) {
    this->text_sensors_.push_back({sensor, source, key});
  }
#endif

  // Writes `data_hex` (an encoded aux region, e.g. from the HA integration's openprinttag.update)
  // to the aux region of the OpenPrintTag with UID `uid`; only the changed blocks are written,
  // then the tag is read again
  void write_aux(const std::string &uid, const std::string &data_hex);

  void on_tag(const std::string &uid) override;
  void on_type5_memory(const std::string &uid, const std::vector<uint8_t> &memory) override;
  void on_tag_removed(const std::string &uid) override;

 protected:
  void clear_();
  void publish_(const OpenPrintTag *tag);

  pn5180::PN5180 *reader_{nullptr};
  std::string uid_;              // of the OpenPrintTag on the reader, empty without one
  std::vector<uint8_t> memory_;  // its memory, for writes
  CallbackManager<void(std::string, std::string)> openprinttag_callback_;

#ifdef USE_SENSOR
  struct SensorField {
    sensor::Sensor *sensor;
    FieldSource source;
    uint32_t key;
  };
  std::vector<SensorField> sensors_;
#endif
#ifdef USE_TEXT_SENSOR
  struct TextSensorField {
    text_sensor::TextSensor *sensor;
    FieldSource source;
    uint32_t key;
  };
  std::vector<TextSensorField> text_sensors_;
#endif
};

}  // namespace esphome::openprinttag

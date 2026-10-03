#include "openprinttag_component.h"

#include <cmath>
#include <cstdio>
#include <iterator>

#include "esphome/core/log.h"

namespace esphome::openprinttag {

static const char *const TAG = "openprinttag";

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

void OpenPrintTagComponent::dump_config() { ESP_LOGCONFIG(TAG, "OpenPrintTag"); }

void OpenPrintTagComponent::on_tag(const std::string &uid) {
  if (!this->uid_.empty() && uid != this->uid_)
    this->clear_();  // another tag replaced the OpenPrintTag
}

void OpenPrintTagComponent::on_type5_memory(const std::string &uid, const std::vector<uint8_t> &memory) {
  OpenPrintTag tag;
  if (!parse_openprinttag(memory, tag)) {
    ESP_LOGD(TAG, "Tag %s holds no OpenPrintTag record", uid.c_str());
    if (uid == this->uid_)
      this->clear_();
    return;
  }
  ESP_LOGD(TAG, "OpenPrintTag %s: %zu main, %zu aux fields", uid.c_str(), tag.main.size(), tag.aux.size());
  this->uid_ = uid;
  this->memory_ = memory;
  this->publish_(&tag);
  this->openprinttag_callback_.call(uid, format_hex(tag.payload.data(), tag.payload.size()));
}

void OpenPrintTagComponent::on_tag_removed(const std::string &uid) {
  if (uid == this->uid_)
    this->clear_();
}

void OpenPrintTagComponent::clear_() {
  const std::string uid = this->uid_;
  this->uid_.clear();
  this->memory_.clear();
  this->publish_(nullptr);
  this->openprinttag_callback_.call(uid, "");
}

void OpenPrintTagComponent::write_aux(const std::string &uid, const std::string &data_hex) {
  OpenPrintTag tag;
  if (uid.empty() || uid != this->uid_ || !parse_openprinttag(this->memory_, tag) || tag.aux_size == 0) {
    ESP_LOGW(TAG, "No OpenPrintTag with an aux region and UID %s on the reader", uid.c_str());
    return;
  }
  std::vector<uint8_t> data(data_hex.size() / 2);
  if (data.empty() || !parse_hex(data_hex, data.data(), data.size()) || data.size() > tag.aux_size) {
    ESP_LOGW(TAG, "Aux data invalid or larger than the aux region (%zu B)", tag.aux_size);
    return;
  }
  // Bytes after the new data stay, readers ignore them
  this->reader_->write(uid, tag.payload_offset + tag.aux_offset, data);
}

void OpenPrintTagComponent::publish_(const OpenPrintTag *tag) {
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
    if (const OptValue *v = find(f.source, f.key); v != nullptr) {
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

}  // namespace esphome::openprinttag

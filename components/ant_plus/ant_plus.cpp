#include "ant_plus.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>
#include <cstring>

extern "C" {
#include <ant_radio.h>
#include <ant_wire.h>
}

namespace esphome::ant_plus {

static const char *const TAG = "ant_plus";

// No page for this long means the sensor is gone (off, asleep, out of range). Covers
// five pages of a 0.5 Hz sensor (Tempe).
static constexpr uint32_t STALE_MS = 10000;
static constexpr uint8_t NETWORK = 0;
static constexpr uint8_t ANT_PLUS_FREQ = 57;  // 2457 MHz
static constexpr uint8_t CHANNEL_TYPE_SLAVE = 0x00;
static constexpr uint8_t EXT_ASSIGN_BACKGROUND_SCAN = 0x01;

// ANT+ common page 82: battery status
static constexpr uint8_t PAGE_BATTERY_STATUS = 0x52;
// HRM page 7: battery status (HRM pages carry a toggle bit in bit 7)
static constexpr uint8_t HRM_PAGE_BATTERY = 7;
// FE-C
static constexpr uint8_t FE_PAGE_GENERAL = 16;
static constexpr uint8_t FE_PAGE_ROWER = 22;
static constexpr uint8_t FE_STATE_IN_USE = 3;
// Environment
static constexpr uint8_t ENV_PAGE_TEMPERATURE = 1;

// antr_on_message() carries no context, so it reaches the instance through this
static AntPlus *global_ant_plus = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

static const char *type_name(DeviceType type) {
  switch (type) {
    case DeviceType::HEART_RATE:
      return "heart rate";
    case DeviceType::FITNESS_EQUIPMENT:
      return "fitness equipment";
    case DeviceType::TEMPERATURE:
      return "temperature";
    case DeviceType::BIKE_RADAR:
      return "bike radar";
    case DeviceType::POWER:
      return "power";
    case DeviceType::SPEED:
      return "speed";
    case DeviceType::SHIFTING:
      return "shifting";
    case DeviceType::ASSET_TRACKER:
      return "asset tracker";
  }
  return "?";
}

static const char *ant_device_type_name(uint8_t type) {
  switch (type & 0x7F) {
    case 11:
      return "power";
    case 17:
      return "fitness equipment";
    case 25:
      return "environment";
    case 34:
      return "shifting";
    case 35:
      return "bike light";
    case 120:
      return "heart rate";
    case 121:
      return "speed and cadence";
    case 122:
      return "cadence";
    case 123:
      return "speed";
    case 124:
      return "stride";
    default:
      return "device";
  }
}

uint8_t AntPlusDevice::ant_device_type() const {
  switch (this->type_) {
    case DeviceType::HEART_RATE:
      return 120;
    case DeviceType::FITNESS_EQUIPMENT:
      return 17;
    case DeviceType::TEMPERATURE:
      return 25;
    case DeviceType::BIKE_RADAR:
      return 40;
    case DeviceType::POWER:
      return 11;
    case DeviceType::SPEED:
      return 123;
    case DeviceType::SHIFTING:
      return 34;
    case DeviceType::ASSET_TRACKER:
      return 41;
  }
  return 0;
}

uint16_t AntPlusDevice::channel_period() const {
  switch (this->type_) {
    case DeviceType::HEART_RATE:
      return 8070;  // 4.06 Hz
    case DeviceType::FITNESS_EQUIPMENT:
      return 8192;  // 4 Hz
    case DeviceType::TEMPERATURE:
      // Environment allows 4 Hz or 0.5 Hz and the channel must match the sensor;
      // Garmin's Tempe runs at 0.5 Hz
      return 65535;
    case DeviceType::BIKE_RADAR:
    case DeviceType::SHIFTING:
      return 8192;  // 4 Hz
    case DeviceType::POWER:
      return 8182;  // 4.005 Hz
    case DeviceType::SPEED:
      return 8118;  // 4.04 Hz
  }
  return 8192;
}

void AntPlusDevice::on_page(const uint8_t *page, uint32_t now) {
  if (!this->connected_state_)
    ESP_LOGI(TAG, "Receiving %s %" PRIu32, type_name(this->type_), this->ant_id_);
  this->last_page_ms_ = now;
  this->set_connected_(true);

  if (page[0] == PAGE_BATTERY_STATUS) {
    if (this->type_ == DeviceType::SHIFTING) {
      this->decode_shifting_battery_(page);
    } else {
      this->decode_battery_(page[6], page[7]);
    }
    return;
  }
  switch (this->type_) {
    case DeviceType::HEART_RATE:
      this->decode_heart_rate_(page);
      break;
    case DeviceType::FITNESS_EQUIPMENT:
      this->decode_fitness_equipment_(page);
      break;
    case DeviceType::TEMPERATURE:
      this->decode_temperature_(page);
      break;
    case DeviceType::POWER:
      this->decode_power_(page);
      break;
    case DeviceType::SPEED:
      this->decode_speed_(page, now);
      break;
    case DeviceType::BIKE_RADAR:
    case DeviceType::SHIFTING:
      break;  // only the battery is read
  }
}

void AntPlusDevice::decode_power_(const uint8_t *page) {
#ifdef USE_SENSOR
  // standard power-only page: [3] instantaneous cadence (rpm), [6-7] instantaneous power (W)
  if (page[0] != 0x10)
    return;
  if (this->cadence_ != nullptr && page[3] != 0xFF)
    this->cadence_->publish_state(page[3]);
  if (this->power_ != nullptr)
    this->power_->publish_state(page[6] | (page[7] << 8));
#endif
}

void AntPlusDevice::decode_speed_(const uint8_t *page, uint32_t now) {
  // every page: [4-5] time of the last wheel event (1/1024 s), [6-7] revolution count
  if ((page[0] & 0x7F) == 4)
    this->decode_battery_(page[2], page[3]);  // [2] fractional V, [3] coarse V + status
  uint16_t event_time = page[4] | (page[5] << 8);
  uint16_t revolutions = page[6] | (page[7] << 8);
  if (this->last_event_time_ < 0) {
    this->last_event_time_ = event_time;
    this->last_revolutions_ = revolutions;
    this->last_revolution_ms_ = now;
    return;
  }
  uint16_t delta_revolutions = revolutions - this->last_revolutions_;
  uint16_t delta_time = event_time - static_cast<uint16_t>(this->last_event_time_);
#ifdef USE_SENSOR
  if (delta_revolutions != 0 && delta_time != 0) {
    this->total_speed_distance_ += delta_revolutions * this->wheel_circumference_;
    if (this->speed_ != nullptr)
      this->speed_->publish_state(delta_revolutions * this->wheel_circumference_ * 1024.0f / delta_time);
    if (this->distance_ != nullptr)
      this->distance_->publish_state(this->total_speed_distance_);
    this->last_revolution_ms_ = now;
    this->stopped_ = false;
  } else if (!this->stopped_ && now - this->last_revolution_ms_ > 3000) {
    // no wheel event for 3 s: the wheel stands still (the sensor keeps sending)
    if (this->speed_ != nullptr)
      this->speed_->publish_state(0);
    this->stopped_ = true;
  }
#endif
  this->last_event_time_ = event_time;
  this->last_revolutions_ = revolutions;
}

void AntPlusDevice::decode_shifting_battery_(const uint8_t *page) {
  // [2] battery identifier (bits 7:4, one per component), [7] bits 6:4 status:
  // 1 new, 2 good, 3 ok, 4 low, 5 critical
  uint8_t id = page[2] >> 4;
  uint8_t status = (page[7] >> 4) & 0x07;
  if (status < 1 || status > 5)
    return;
  if (this->battery_status_[id] != status)
    ESP_LOGI(TAG, "Shifting %" PRIu32 " battery %u: status %u", this->ant_id_, id, status);
  this->battery_status_[id] = status;
#ifdef USE_BINARY_SENSOR
  bool low = false;
  for (uint8_t s : this->battery_status_)
    low |= s >= 4;
  if (this->battery_low_ != nullptr)
    this->battery_low_->publish_state(low);
#endif
}

void AntPlusDevice::decode_heart_rate_(const uint8_t *page) {
  uint8_t number = page[0] & 0x7F;
  if (number >= 0x10)
    return;  // not an HRM page
#ifdef USE_SENSOR
  // byte 7 of every HRM page is the computed heart rate
  if (this->heart_rate_ != nullptr)
    this->heart_rate_->publish_state(page[7]);
  if (number == HRM_PAGE_BATTERY) {
    if (this->battery_level_ != nullptr && page[1] <= 100)
      this->battery_level_->publish_state(page[1]);
    this->decode_battery_(page[2], page[3]);
  }
#endif
}

void AntPlusDevice::decode_fitness_equipment_(const uint8_t *page) {
  if (page[0] < FE_PAGE_GENERAL || page[0] > 25)
    return;
  // bits 4-6 of byte 7 are the FE state on every FE-specific page
  uint8_t state = (page[7] >> 4) & 0x07;
#ifdef USE_BINARY_SENSOR
  if (this->in_use_ != nullptr)
    this->in_use_->publish_state(state == FE_STATE_IN_USE);
#endif
#ifdef USE_SENSOR
  if (page[0] == FE_PAGE_GENERAL) {
    // [3] distance in m (rolls over at 256), [4-5] speed in 0.001 m/s
    if (this->last_distance_ >= 0)
      this->total_distance_ += (page[3] - this->last_distance_) & 0xFF;
    this->last_distance_ = page[3];
    if (this->distance_ != nullptr)
      this->distance_->publish_state(this->total_distance_);
    uint16_t speed = page[4] | (page[5] << 8);
    if (this->speed_ != nullptr && speed != 0xFFFF)
      this->speed_->publish_state(speed * 0.001f);
    if (this->heart_rate_ != nullptr && page[6] != 0xFF)
      this->heart_rate_->publish_state(page[6]);
  } else if (page[0] == FE_PAGE_ROWER) {
    // [4] stroke rate (strokes/min), [5-6] instantaneous power in W
    if (this->cadence_ != nullptr && page[4] != 0xFF)
      this->cadence_->publish_state(page[4]);
    uint16_t power = page[5] | (page[6] << 8);
    if (this->power_ != nullptr && power != 0xFFFF)
      this->power_->publish_state(power);
  }
#endif
}

void AntPlusDevice::decode_temperature_(const uint8_t *page) {
#ifdef USE_SENSOR
  if (page[0] != ENV_PAGE_TEMPERATURE)
    return;
  // [3-5] 24 h low and high, two signed 12-bit values in 0.1 degC (0x800 = invalid),
  // packed in opposite directions around byte 4: low = [3] + [4] bits 7:4 on top,
  // high = [4] bits 3:0 at the bottom + [5]
  auto publish_12bit = [](sensor::Sensor *s, uint16_t raw) {
    if (s == nullptr || raw == 0x800)
      return;
    int16_t value = raw & 0x800 ? static_cast<int16_t>(raw | 0xF000) : static_cast<int16_t>(raw);
    s->publish_state(value * 0.1f);
  };
  publish_12bit(this->temperature_min_, page[3] | ((page[4] >> 4) << 8));
  publish_12bit(this->temperature_max_, (page[4] & 0x0F) | (page[5] << 4));
  // [6-7] current temperature, signed, 0.01 degC
  int16_t temperature = static_cast<int16_t>(page[6] | (page[7] << 8));
  if (this->temperature_ != nullptr && temperature != INT16_MIN)
    this->temperature_->publish_state(temperature * 0.01f);
#endif
}

void AntPlusDevice::decode_battery_(uint8_t fractional, uint8_t coarse) {
#ifdef USE_SENSOR
  // coarse volts in bits 0-3 (0xF = invalid), fraction in 1/256 V
  if (this->battery_voltage_ != nullptr && (coarse & 0x0F) != 0x0F)
    this->battery_voltage_->publish_state((coarse & 0x0F) + fractional / 256.0f);
#endif
#ifdef USE_BINARY_SENSOR
  // status in bits 4-6: 1 new, 2 good, 3 ok, 4 low, 5 critical (0, 6, 7 = none)
  uint8_t status = (coarse >> 4) & 0x07;
  if (this->battery_low_ != nullptr && status >= 1 && status <= 5)
    this->battery_low_->publish_state(status >= 4);
#endif
}

void AntPlusDevice::check_stale(uint32_t now) {
  if (!this->connected_state_ || now - this->last_page_ms_ <= STALE_MS)
    return;
  ESP_LOGI(TAG, "Lost %s %" PRIu32, type_name(this->type_), this->ant_id_);
  this->set_connected_(false);
#ifdef USE_SENSOR
  for (auto *s : {this->heart_rate_, this->power_, this->cadence_, this->speed_, this->temperature_,
                  this->temperature_min_, this->temperature_max_}) {
    if (s != nullptr)
      s->publish_state(NAN);
  }
#endif
#ifdef USE_BINARY_SENSOR
  if (this->in_use_ != nullptr)
    this->in_use_->publish_state(false);
#endif
}

void AntPlusDevice::set_connected_(bool connected) {
  this->connected_state_ = connected;
#ifdef USE_BINARY_SENSOR
  if (this->connected_ != nullptr)
    this->connected_->publish_state(connected);
#endif
}

uint8_t AntPlusTransmitter::ant_device_type() const {
  switch (this->type_) {
    case DeviceType::TEMPERATURE:
      return 25;
    case DeviceType::ASSET_TRACKER:
      return 41;
    default:
      return 0;
  }
}

uint16_t AntPlusTransmitter::channel_period() const {
  // Environment: 0.5 Hz like a Garmin Tempe, what watches expect from a temperature
  // sensor. Tracker: 16 Hz, the profile's only rate.
  return this->type_ == DeviceType::ASSET_TRACKER ? 2048 : 65535;
}

uint8_t AntPlusTransmitter::transmission_type() const {
  // 0x05: independent channel, global data pages; the top 4 bits of a 20-bit ANT ID
  // go in the upper nibble
  return 0x05 | ((this->ant_id_ >> 12) & 0xF0);
}

#ifdef USE_SENSOR
void AntPlusTransmitter::set_position(sensor::Sensor *latitude, sensor::Sensor *longitude) {
  latitude->add_on_state_callback([this](float value) { this->latitude_ = value; });
  longitude->add_on_state_callback([this](float value) { this->longitude_ = value; });
}

void AntPlusTransmitter::add_asset(const char *name, uint8_t colour, uint8_t asset_type, sensor::Sensor *latitude,
                                   sensor::Sensor *longitude) {
  Asset asset{};
  std::strncpy(asset.name, name, sizeof(asset.name));  // not null terminated at 10 chars
  asset.colour = colour;
  asset.asset_type = asset_type;
  this->assets_.push_back(asset);
  size_t index = this->assets_.size() - 1;
  latitude->add_on_state_callback([this, index](float value) { this->assets_[index].latitude = value; });
  longitude->add_on_state_callback([this, index](float value) { this->assets_[index].longitude = value; });
}

void AntPlusTransmitter::set_source(sensor::Sensor *source) {
  source->add_on_state_callback([this](float value) {
    this->value_ = value;
    this->event_count_++;
    if (std::isnan(value))
      return;
    uint32_t hour = millis() / 3600000;
    Hour &bucket = this->hours_[hour % 24];
    if (bucket.hour != hour)
      bucket = Hour{hour, value, value};
    bucket.low = std::min(bucket.low, value);
    bucket.high = std::max(bucket.high, value);
  });
}
#endif

void AntPlusTransmitter::next_page(uint8_t *page) {
  uint32_t n = this->message_count_++;
  if (this->type_ == DeviceType::ASSET_TRACKER) {
    this->next_tracker_page_(n, page);
  } else {
    this->next_environment_page_(n, page);
  }
}

void AntPlusTransmitter::next_tracker_page_(uint32_t n, uint8_t *page) {
  // common pages 80 (manufacturer) and 81 (product) every 65 messages
  if (n % 65 == 64) {
    const uint8_t p80[8] = {0x50, 0xFF, 0xFF, 1, 0xFF, 0x00, 1, 0};
    const uint8_t p81[8] = {0x51, 0xFF, 0xFF, 10, static_cast<uint8_t>(this->ant_id_),
                            static_cast<uint8_t>(this->ant_id_ >> 8), static_cast<uint8_t>(this->ant_id_ >> 16), 0};
    std::memcpy(page, n % 130 < 65 ? p80 : p81, 8);
    return;
  }
  if (this->assets_.empty()) {
    const uint8_t p3[8] = {0x03, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // no assets
    std::memcpy(page, p3, 8);
    return;
  }
  // per 20 messages: 9 location pairs (pages 1 + 2) and one identification pair (16 + 17),
  // rotating over the assets
  uint32_t slot = n % 20;
  uint8_t index = slot < 18 ? (n / 2) % this->assets_.size() : (n / 20) % this->assets_.size();
  const Asset &asset = this->assets_[index];
  uint8_t index_byte = 0xE0 | index;  // reserved top 3 bits set
  if (slot >= 18) {
    page[0] = slot == 18 ? 0x10 : 0x11;
    page[1] = index_byte;
    page[2] = slot == 18 ? asset.colour : asset.asset_type;  // 0 tracker, 1 dog
    std::memcpy(page + 3, asset.name + (slot == 18 ? 0 : 5), 5);
    return;
  }
  bool has_position = !std::isnan(asset.latitude) && !std::isnan(asset.longitude);
  // semicircles: degrees * 2^31 / 180
  int32_t latitude = has_position ? static_cast<int32_t>(llround(asset.latitude * (2147483648.0 / 180.0))) : 0;
  int32_t longitude = has_position ? static_cast<int32_t>(llround(asset.longitude * (2147483648.0 / 180.0))) : 0;
  if (slot % 2 == 0) {
    // page 1: distance (m) and bearing (1/256 turn) from the tracker's own position,
    // which watches show as they are; status: situation "unknown" (4), GPS lost if no position
    uint16_t distance = 0;
    uint8_t bearing = 0;
    if (has_position && !std::isnan(this->latitude_) && !std::isnan(this->longitude_)) {
      constexpr double RAD = M_PI / 180.0;
      double lat1 = this->latitude_ * RAD, lat2 = asset.latitude * RAD;
      double dlat = lat2 - lat1, dlon = (asset.longitude - this->longitude_) * RAD;
      double a = sin(dlat / 2) * sin(dlat / 2) + cos(lat1) * cos(lat2) * sin(dlon / 2) * sin(dlon / 2);
      distance = static_cast<uint16_t>(std::min(65535.0, 2 * 6371000.0 * atan2(sqrt(a), sqrt(1 - a))));
      double b = atan2(sin(dlon) * cos(lat2), cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon));
      bearing = static_cast<uint8_t>(lround(fmod(b / RAD + 360.0, 360.0) * 256.0 / 360.0) & 0xFF);
    }
    uint8_t status = 0x04 | (has_position ? 0 : 0x10);
    const uint8_t p1[8] = {0x01,
                           index_byte,
                           static_cast<uint8_t>(distance),
                           static_cast<uint8_t>(distance >> 8),
                           bearing,
                           status,
                           static_cast<uint8_t>(latitude),
                           static_cast<uint8_t>(latitude >> 8)};
    std::memcpy(page, p1, 8);
  } else {
    const uint8_t p2[8] = {0x02,
                           index_byte,
                           static_cast<uint8_t>(latitude >> 16),
                           static_cast<uint8_t>(latitude >> 24),
                           static_cast<uint8_t>(longitude),
                           static_cast<uint8_t>(longitude >> 8),
                           static_cast<uint8_t>(longitude >> 16),
                           static_cast<uint8_t>(longitude >> 24)};
    std::memcpy(page, p2, 8);
  }
}

void AntPlusTransmitter::next_environment_page_(uint32_t n, uint8_t *page) {
  // common pages 80 (manufacturer) and 81 (product) at least every 65 messages
  if (n % 65 == 63 || n % 65 == 64) {
    if (n % 130 < 65) {
      // manufacturer 255 = development, hardware revision 1, model 1
      const uint8_t p80[8] = {0x50, 0xFF, 0xFF, 1, 0xFF, 0x00, 1, 0};
      std::memcpy(page, p80, 8);
    } else {
      // software 1.0, serial number = ANT ID
      const uint8_t p81[8] = {0x51,
                              0xFF,
                              0xFF,
                              10,
                              static_cast<uint8_t>(this->ant_id_),
                              static_cast<uint8_t>(this->ant_id_ >> 8),
                              static_cast<uint8_t>(this->ant_id_ >> 16),
                              0};
      std::memcpy(page, p81, 8);
    }
    return;
  }
  if (n % 32 == 31) {
    // Environment page 0: 0.5 Hz, no clocks, pages 0 and 1 supported
    const uint8_t p0[8] = {0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00};
    std::memcpy(page, p0, 8);
    return;
  }
  // Environment page 1: event count, 24 h low/high (signed 12 bit, 0.1 degC, 0x800 =
  // unknown), current in 0.01 degC
  float low = NAN, high = NAN;
  uint32_t now_hour = millis() / 3600000;
  for (const Hour &bucket : this->hours_) {
    if (bucket.hour == UINT32_MAX || now_hour - bucket.hour >= 24)
      continue;
    low = std::isnan(low) ? bucket.low : std::min(low, bucket.low);
    high = std::isnan(high) ? bucket.high : std::max(high, bucket.high);
  }
  auto to_12bit = [](float value) -> uint16_t {
    return std::isnan(value) ? 0x800 : static_cast<uint16_t>(lroundf(value * 10)) & 0xFFF;
  };
  uint16_t low12 = to_12bit(low), high12 = to_12bit(high);
  int16_t temperature = std::isnan(this->value_) ? INT16_MIN : static_cast<int16_t>(lroundf(this->value_ * 100));
  // low = [3] + [4] bits 7:4 on top, high = [4] bits 3:0 at the bottom + [5]
  const uint8_t p1[8] = {0x01,
                         0xFF,
                         this->event_count_,
                         static_cast<uint8_t>(low12 & 0xFF),
                         static_cast<uint8_t>(((low12 >> 8) << 4) | (high12 & 0x0F)),
                         static_cast<uint8_t>(high12 >> 4),
                         static_cast<uint8_t>(temperature & 0xFF),
                         static_cast<uint8_t>((temperature >> 8) & 0xFF)};
  std::memcpy(page, p1, 8);
}

void AntPlus::setup() {
  global_ant_plus = this;

  antr_err_t err = antr_init();
  if (err == 0)
    err = antr_network_address_set(NETWORK, this->network_key_.data());
  if (err != 0) {
    ESP_LOGE(TAG, "ANT setup failed: 0x%02X", err);
    this->mark_failed();
    return;
  }
  for (uint8_t channel = 0; channel < this->devices_.size(); channel++) {
    auto *device = this->devices_[channel];
    // the device number is the low 16 bits of the ANT ID (0 = any). The top 4 bits
    // ride in the transmission type's upper nibble, but its lower nibble is the
    // sensor's own, so the transmission type stays a wildcard
    err = antr_channel_assign(channel, CHANNEL_TYPE_SLAVE, NETWORK, 0);
    if (err == 0)
      err = antr_channel_id_set(channel, device->get_ant_id() & 0xFFFF, device->ant_device_type(), 0);
    if (err == 0)
      err = antr_channel_period_set(channel, device->channel_period());
    if (err == 0)
      err = antr_channel_radio_freq_set(channel, ANT_PLUS_FREQ);
    if (err != 0 || !this->open_channel_(channel)) {
      ESP_LOGE(TAG, "Channel %u setup failed: 0x%02X", channel, err);
      this->mark_failed();
      return;
    }
  }
#ifdef USE_BINARY_SENSOR
  if (this->unknown_device_ != nullptr)
    this->discovery_ = this->setup_discovery_channel_(this->devices_.size());
#endif

  this->first_transmitter_channel_ = this->devices_.size() + (this->discovery_ ? 1 : 0);
  for (uint8_t i = 0; i < this->transmitters_.size(); i++) {
    auto *tx = this->transmitters_[i];
    uint8_t channel = this->first_transmitter_channel_ + i;
    uint8_t page[8];
    tx->next_page(page);
    err = antr_channel_assign(channel, ANTW_CHANNEL_TYPE_MASTER, NETWORK, 0);
    if (err == 0)
      err = antr_channel_id_set(channel, tx->get_ant_id() & 0xFFFF, tx->ant_device_type(), tx->transmission_type());
    if (err == 0)
      err = antr_channel_period_set(channel, tx->channel_period());
    if (err == 0)
      err = antr_channel_radio_freq_set(channel, ANT_PLUS_FREQ);
    if (err == 0 && this->open_channel_(channel))
      err = antr_broadcast_message_tx(channel, 8, page);
    if (err != 0)
      ESP_LOGW(TAG, "Transmitter channel %u setup failed: 0x%02X", channel, err);
  }
}

bool AntPlus::setup_discovery_channel_(uint8_t channel) {
  // a background scan channel doesn't pair: it hears every ANT+ device on the frequency,
  // and with the device ID appended to each message the configured ones are filtered out
  antr_err_t err = antr_lib_config_set(ANTW_LIB_CONFIG_MESG_OUT_INC_DEVICE_ID);
  if (err == 0)
    err = antr_channel_assign(channel, CHANNEL_TYPE_SLAVE, NETWORK, EXT_ASSIGN_BACKGROUND_SCAN);
  if (err == 0)
    err = antr_channel_id_set(channel, 0, 0, 0);
  if (err == 0)
    err = antr_channel_radio_freq_set(channel, ANT_PLUS_FREQ);
  if (err != 0) {
    ESP_LOGW(TAG, "Discovery channel setup failed: 0x%02X", err);
    return false;
  }
  return this->open_channel_(channel);
}

void AntPlus::on_discovery_message_(const Message &msg, uint32_t now) {
  uint16_t device_number = msg.device_id[0] | (msg.device_id[1] << 8);
  if (device_number == 0 && msg.device_id[2] == 0)
    return;  // no extended data
  // a known device's other profiles (the HRM-Pro's stride channel) aren't unknown either
  for (auto *device : this->devices_) {
    if ((device->get_ant_id() & 0xFFFF) == device_number)
      return;
  }
  uint32_t ant_id = (uint32_t(msg.device_id[3] & 0xF0) << 12) | device_number;
  this->last_unknown_ms_ = now;
  this->unknown_state_ = true;
  if (std::find(this->unknown_ids_.begin(), this->unknown_ids_.end(), ant_id) == this->unknown_ids_.end()) {
    if (this->unknown_ids_.size() < 16)
      this->unknown_ids_.push_back(ant_id);
    ESP_LOGI(TAG, "Unknown ANT+ %s in range, ANT ID %" PRIu32 " (device type %u)",
             ant_device_type_name(msg.device_id[2]), ant_id, msg.device_id[2] & 0x7F);
  }
}

bool AntPlus::open_channel_(uint8_t channel) {
  antr_err_t err = antr_channel_open_with_offset(channel, 0);
  if (err != 0)
    ESP_LOGW(TAG, "Opening channel %u failed: 0x%02X", channel, err);
  return err == 0;
}

void AntPlus::loop() {
  uint32_t now = millis();
  uint8_t tail = this->tail_.load(std::memory_order_relaxed);
  while (tail != this->head_.load(std::memory_order_acquire)) {
    const Message &msg = this->queue_[tail % QUEUE_SIZE];
    bool discovery = this->discovery_ && msg.channel == this->devices_.size();
    if (msg.channel >= this->first_transmitter_channel_ &&
        msg.channel < this->first_transmitter_channel_ + this->transmitters_.size()) {
      // the slot fired: queue the next page
      if (msg.id == ANTW_MESG_RESPONSE_EVENT_ID && msg.data[0] == ANTW_EVENT_TX) {
        uint8_t page[8];
        this->transmitters_[msg.channel - this->first_transmitter_channel_]->next_page(page);
        antr_broadcast_message_tx(msg.channel, 8, page);
      }
    } else if (msg.channel < this->devices_.size() || discovery) {
      if (msg.id == ANTW_MESG_RESPONSE_EVENT_ID) {
        // search timed out and closed the channel: keep looking
        if (msg.data[0] == ANTW_EVENT_CHANNEL_CLOSED)
          this->open_channel_(msg.channel);
      } else if (discovery) {
        this->on_discovery_message_(msg, now);
      } else {
        this->devices_[msg.channel]->on_page(msg.data, now);
      }
    }
    tail++;
    this->tail_.store(tail, std::memory_order_release);
  }
  bool any_connected = false;
  for (auto *device : this->devices_) {
    device->check_stale(now);
    any_connected |= device->is_connected();
  }
  if (this->unknown_state_ && now - this->last_unknown_ms_ > STALE_MS)
    this->unknown_state_ = false;
#ifdef USE_BINARY_SENSOR
  if (this->connected_ != nullptr)
    this->connected_->publish_state(any_connected);
  if (this->unknown_device_ != nullptr)
    this->unknown_device_->publish_state(this->unknown_state_);
#endif
  if (uint32_t dropped = this->dropped_.exchange(0); dropped != 0)
    ESP_LOGW(TAG, "%" PRIu32 " ANT messages dropped", dropped);
}

void AntPlus::dump_config() {
  ESP_LOGCONFIG(TAG, "ANT+ (RadiANT):");
  for (auto *device : this->devices_)
    ESP_LOGCONFIG(TAG, "  %s, ANT ID %" PRIu32 "%s", type_name(device->get_type()), device->get_ant_id(),
                  device->get_ant_id() == 0 ? " (any)" : "");
  for (auto *tx : this->transmitters_)
    ESP_LOGCONFIG(TAG, "  Transmitting %s as ANT ID %" PRIu32, type_name(tx->get_type()), tx->get_ant_id());
}

void AntPlus::on_message(uint8_t id, const uint8_t *data, uint8_t len) {
  Message msg{};
  if ((id == ANTW_MESG_BROADCAST_DATA_ID || id == ANTW_MESG_ACKNOWLEDGED_DATA_ID) && len >= 9) {
    msg.id = id;
    std::memcpy(msg.data, data + 1, 8);
    // [9] flag byte, 0x80 = device ID follows in [10-13]
    if (len >= 14 && (data[9] & 0x80))
      std::memcpy(msg.device_id, data + 10, 4);
  } else if (id == ANTW_MESG_RESPONSE_EVENT_ID && len >= 3 && data[1] == ANTW_MESG_EVENT_ID &&
             (data[2] == ANTW_EVENT_CHANNEL_CLOSED || data[2] == ANTW_EVENT_TX)) {
    msg.id = id;
    msg.data[0] = data[2];
  } else {
    return;
  }
  msg.channel = data[0];
  uint8_t head = this->head_.load(std::memory_order_relaxed);
  if (static_cast<uint8_t>(head - this->tail_.load(std::memory_order_acquire)) >= QUEUE_SIZE) {
    this->dropped_.fetch_add(1);
    return;
  }
  this->queue_[head % QUEUE_SIZE] = msg;
  this->head_.store(head + 1, std::memory_order_release);
}

}  // namespace esphome::ant_plus

// RadiANT hands every message up through this, resolved at link time
extern "C" void antr_on_message(const struct antr_msg *msg) {
  if (esphome::ant_plus::global_ant_plus != nullptr)
    esphome::ant_plus::global_ant_plus->on_message(msg->id, msg->data, msg->len);
}

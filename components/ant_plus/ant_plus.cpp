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
    if (msg.channel < this->devices_.size() || discovery) {
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
             data[2] == ANTW_EVENT_CHANNEL_CLOSED) {
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

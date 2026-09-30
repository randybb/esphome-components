#include "ant_plus.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <cinttypes>
#include <cmath>

extern "C" {
#include <ant_radio.h>
#include <ant_wire.h>
}

namespace esphome::ant_plus {

static const char *const TAG = "ant_plus";

// No page for this long means the monitor is gone (off, out of range, no skin contact)
static constexpr uint32_t STALE_MS = 5000;
static constexpr uint8_t CHANNEL = 0;
static constexpr uint8_t NETWORK = 0;
// ANT+ heart rate monitor: device type 120, 4.06 Hz, 2457 MHz
static constexpr uint8_t HRM_DEVICE_TYPE = 0x78;
static constexpr uint16_t HRM_PERIOD = 8070;
static constexpr uint8_t ANT_PLUS_FREQ = 57;
static constexpr uint8_t CHANNEL_TYPE_SLAVE = 0x00;

// antr_on_message() carries no context, so it reaches the instance through this
static AntPlus *global_ant_plus = nullptr;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void AntPlus::setup() {
  global_ant_plus = this;

  antr_err_t err = antr_init();
  if (err == 0)
    err = antr_network_address_set(NETWORK, this->network_key_.data());
  if (err == 0)
    err = antr_channel_assign(CHANNEL, CHANNEL_TYPE_SLAVE, NETWORK, 0);
  // the device number is the low 16 bits of the ANT ID (0 = any). The top 4 bits ride
  // in the transmission type's upper nibble, but its lower nibble is the sensor's own
  // (0x01 for HRM), so the transmission type stays a wildcard
  if (err == 0)
    err = antr_channel_id_set(CHANNEL, this->device_number_ & 0xFFFF, HRM_DEVICE_TYPE, 0);
  if (err == 0)
    err = antr_channel_period_set(CHANNEL, HRM_PERIOD);
  if (err == 0)
    err = antr_channel_radio_freq_set(CHANNEL, ANT_PLUS_FREQ);
  if (err == 0)
    err = antr_channel_open_with_offset(CHANNEL, 0);
  if (err != 0) {
    ESP_LOGE(TAG, "ANT setup failed: 0x%02X", err);
    this->mark_failed();
  }
}

void AntPlus::loop() {
  if (uint8_t event = this->last_event_.exchange(0); event != 0)
    ESP_LOGD(TAG, "Channel event 0x%02X", event);
  if (this->reopen_.exchange(false)) {
    // the search timed out and closed the channel: keep looking for the monitor
    antr_err_t err = antr_channel_open_with_offset(CHANNEL, 0);
    if (err != 0)
      ESP_LOGW(TAG, "Reopening the channel failed: 0x%02X", err);
  }

  if (this->have_data_.exchange(false)) {
    if (this->published_nan_) {
      uint16_t device_number;
      uint8_t device_type, trans_type;
      if (antr_channel_id_get(CHANNEL, &device_number, &device_type, &trans_type) == 0)
        this->paired_id_ = (uint32_t(trans_type & 0xF0) << 12) | device_number;
      ESP_LOGI(TAG, "Receiving heart rate monitor %" PRIu32, this->paired_id_);
    }
    this->published_nan_ = false;
    if (this->heart_rate_sensor_ != nullptr)
      this->heart_rate_sensor_->publish_state(this->heart_rate_.load());
  } else if (!this->published_nan_ && millis() - this->last_page_ms_.load() > STALE_MS) {
    ESP_LOGI(TAG, "Heart rate monitor lost");
    if (this->heart_rate_sensor_ != nullptr)
      this->heart_rate_sensor_->publish_state(NAN);
    this->published_nan_ = true;
  }
}

void AntPlus::dump_config() {
  ESP_LOGCONFIG(TAG,
                "ANT+ (RadiANT):\n"
                "  Heart rate monitor ANT ID: %" PRIu32 "%s",
                this->device_number_, this->device_number_ == 0 ? " (any)" : "");
  LOG_SENSOR("  ", "Heart Rate", this->heart_rate_sensor_);
}

void AntPlus::on_message(uint8_t id, const uint8_t *data, uint8_t len) {
  if (len < 1 || data[0] != CHANNEL)
    return;
  if ((id == ANTW_MESG_BROADCAST_DATA_ID || id == ANTW_MESG_ACKNOWLEDGED_DATA_ID) && len >= 9) {
    // byte 7 of every ANT+ HRM page is the computed heart rate
    this->heart_rate_.store(data[8]);
    this->last_page_ms_.store(millis());
    this->have_data_.store(true);
  } else if (id == ANTW_MESG_RESPONSE_EVENT_ID && len >= 3 && data[1] == ANTW_MESG_EVENT_ID) {
    if (data[2] != ANTW_EVENT_RX_FAIL)
      this->last_event_.store(data[2]);
    if (data[2] == ANTW_EVENT_CHANNEL_CLOSED)
      this->reopen_.store(true);
  }
}

}  // namespace esphome::ant_plus

// RadiANT hands every message up through this, resolved at link time
extern "C" void antr_on_message(const struct antr_msg *msg) {
  if (esphome::ant_plus::global_ant_plus != nullptr)
    esphome::ant_plus::global_ant_plus->on_message(msg->id, msg->data, msg->len);
}

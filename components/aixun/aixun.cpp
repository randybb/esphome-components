#include "aixun.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"

#ifdef USE_WIFI
#include "esphome/components/wifi/wifi_component.h"
#endif

#include <cstdio>
#include <cstring>

namespace esphome::aixun {

ESPHOME_LOG_TAG(TAG, "aixun");

void Aixun::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  if (this->h310d_) {
    this->h310d_loop_(now);
    return;
  }
  bool connected = this->parent_->is_connected();
  // Read again after a replug, the firmware may have been updated meanwhile.
  if (!connected)
    this->have_version_ = false;
#ifdef USE_TEXT_SENSOR
  if (connected && !this->have_version_ && (this->device_model_ != nullptr || this->firmware_version_ != nullptr) &&
      now - this->last_version_request_ > 2000) {
    this->send_("JC_version");
    this->last_version_request_ = now;
  }
#endif
  // (Re)open whenever the stream stops: USB just enumerated, or the supply was power cycled.
  if (this->p2408_ && connected && now - this->last_status_ > 2000 && now - this->last_open_ > 2000) {
    if (this->opened_)
      ESP_LOGW(TAG, "No status, reopening");
    this->send_("JCXX_open");
    this->send_("JC_Read_Set");
    this->last_open_ = now;
    this->opened_ = true;
  }

  this->read_available([this, now](uint8_t c) {
    this->last_byte_ = now;
    if (char *line = this->line_reader_.feed(c))
      this->handle_line_(line);
  });
  // The JC_version reply has no line end, it is complete once the line goes quiet.
  if (this->line_reader_.length() > 0 && now - this->last_byte_ > 50) {
    char line[160];
    strcpy(line, this->line_reader_.partial());  // handle_line_() cuts the line up
    this->line_reader_.reset();
    this->handle_line_(line);
  }
}

void Aixun::handle_line_(char *line) {
  // JC_M_P2408_version1.27: no line end, so on a P2408 the next status line follows on the same line.
  if (char *v = strstr(line, "_version")) {
    char *model = line;
    for (char *p = line; p < v; p++) {
      if (strncmp(p, "JC_", 3) == 0)
        model = p;
    }
    *v = '\0';
    char *ver = v + strlen("_version");
    ver[strcspn(ver, "{")] = '\0';
    ESP_LOGI(TAG, "Model %s, firmware %s", model, ver);
#ifdef USE_TEXT_SENSOR
    if (this->device_model_ != nullptr)
      this->device_model_->publish_state(model);
    if (this->firmware_version_ != nullptr)
      this->firmware_version_->publish_state(ver);
#endif
    this->have_version_ = true;
    return;
  }
  // A status line can be cut short and continued by the next one on the same line, the last '{' starts the full one.
  if (const char *p = strrchr(line, '{')) {
    int work, channel, o1, o2, o3;
    float v[6];
    if (sscanf(p, "{work:%d,channel:%d,out:%d,%d,%d,V:%f,I:%f,V:%f,I:%f,V:%f,I:%f}", &work, &channel, &o1, &o2, &o3,
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 11) {
      for (int i = 0; i < 3; i++) {
        this->vi_[i][0] = v[2 * i];
        this->vi_[i][1] = v[2 * i + 1];
      }
      this->fresh_ = true;
      this->last_status_ = App.get_loop_component_start_time();
    }
    return;
  }
  // JC_Set:DC2.500,BATT3.000,DCCV,BATTCV
  if (const char *p = strstr(line, "JC_Set:")) {
    float dc, batt;
    if (sscanf(p, "JC_Set:DC%f,BATT%f", &dc, &batt) == 2) {
      ESP_LOGD(TAG, "Current limits: DC %.3f A, Boot %.3f A", dc, batt);
      this->batt_limit_ = batt;
#ifdef USE_NUMBER
      if (this->dc_limit_number_ != nullptr)
        this->dc_limit_number_->publish_state(dc);
#endif
    }
    return;
  }
  if (line[0] != '\0')
    ESP_LOGV(TAG, "Other: %s", line);
}

void Aixun::update() {
  if (this->h310d_) {
    this->h310d_update_();
    return;
  }
#ifdef USE_SENSOR
  for (int i = 0; i < 3; i++) {
    float v = this->fresh_ ? this->vi_[i][0] : NAN;
    float a = this->fresh_ ? this->vi_[i][1] : NAN;
    float values[3] = {v, a, v * a};
    for (int j = 0; j < 3; j++) {
      if (this->sensors_[i][j] != nullptr)
        this->sensors_[i][j]->publish_state(values[j]);
    }
  }
#endif
  this->fresh_ = false;
}

void Aixun::set_dc_output(bool on) { this->send_(on ? "JC_Out_Open:3" : "JC_Out_Close:3"); }

bool Aixun::set_dc_current_limit(float amps) {
  if (std::isnan(this->batt_limit_)) {
    ESP_LOGW(TAG, "Current limits not read yet");
    return false;
  }
  // The supply parses fixed offsets, both values must be x.xxx.
  char cmd[40];
  snprintf(cmd, sizeof(cmd), "JC_Set:DC%.3f,BATT%.3f", clamp(amps, 0.0f, 8.0f), this->batt_limit_);
  this->send_(cmd);
  this->send_("JC_Read_Set");
  return true;
}

void Aixun::send_(const char *cmd) {
  if (!this->parent_->is_connected())
    return;
  ESP_LOGD(TAG, "> %s", cmd);
  this->write_str(cmd);
  this->write_str("\r\n");
}

void Aixun::h310d_set_temperature(int celsius) {
  const uint8_t pl[4] = {0x3c, 2, static_cast<uint8_t>(celsius), static_cast<uint8_t>(celsius >> 8)};
  this->h310d_send_(0x04, pl, sizeof(pl));
}

void Aixun::h310d_set_fan(int speed) {
  const uint8_t pl[3] = {0x3d, 1, static_cast<uint8_t>(speed)};
  this->h310d_send_(0x04, pl, sizeof(pl));
}

void Aixun::h310d_send_connect_() {
  // Pair as an Aixun BS08 remote: TLV 0x31 = name "BS08_<MAC>", TLV 0x30 = connect. The station then pushes its
  // status (cmd 0x02, TLV 0x39) on every change. A new name is confirmed once on the panel ("BS08 connecting").
  uint8_t mac[6];
  get_mac_address_raw(mac);
  char name[18];  // "BS08_" + 12 hex digits, formatted like the remote's own firmware
  snprintf(name, sizeof(name), "BS08_%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  const uint8_t n = 17;
  uint8_t pl[2 + n + 3];
  pl[0] = 0x31;
  pl[1] = n;
  memcpy(pl + 2, name, n);
  pl[2 + n] = 0x30;
  pl[3 + n] = 1;
  pl[4 + n] = 1;
  this->h310d_send_(0x04, pl, 5 + n);
}

void Aixun::h310d_recall_preset(int m) {
  // TLV 0x35/0x36/0x37 = keys M1/M2/M3, second value byte < 8 = short press (recall)
  const uint8_t pl[4] = {static_cast<uint8_t>(0x34 + m), 2, 0, 0};
  this->h310d_send_(0x04, pl, sizeof(pl));
}

uint8_t Aixun::crc8_(const uint8_t *d, uint16_t n) {
  uint8_t c = 0;
  for (uint16_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++)
      c = (c & 0x80) ? static_cast<uint8_t>((c << 1) ^ 0x07) : static_cast<uint8_t>(c << 1);
  }
  return c;
}

void Aixun::h310d_send_(uint8_t cmd, const uint8_t *payload, uint8_t plen) {
  uint8_t f[72];
  uint16_t total = plen + 8;  // 7-byte header + payload + 1 crc
  if (total > sizeof(f))
    return;
  f[0] = cmd;
  f[1] = 0x00;  // request to the MCU
  f[2] = total & 0xFF;
  f[3] = (total >> 8) & 0xFF;
  f[4] = f[5] = f[6] = 0;
  for (uint8_t i = 0; i < plen; i++)
    f[7 + i] = payload[i];
  f[total - 1] = crc8_(f, total - 1);
  this->write_array(f, total);
}

void Aixun::h310d_dispatch_(const uint8_t *f, uint16_t len) {
  const uint8_t cmd = f[0];
  const uint8_t *pl = f + 7;
  const uint16_t pll = len - 8;  // payload length
  if (cmd == 0x01 && f[1] == 0xAA) {
    // device-info reply: TLV 0x01 model, 0x02 serial, 0x03 firmware
    uint16_t p = 0;
    while (p + 2 <= pll) {
      const uint8_t tag = pl[p], l = pl[p + 1];
      if (p + 2 + l > pll)
        break;
      std::string v(reinterpret_cast<const char *>(&pl[p + 2]), l);
      if (tag == 0x01)
        this->h310d_model_ = v;
      else if (tag == 0x02)
        this->h310d_serial_ = v;
      else if (tag == 0x03)
        this->h310d_fw_ = v;
      p += 2 + l;
    }
    this->h310d_info_dirty_ = true;
    return;
  }
  if (cmd == 0x04) {
    // MCU UI-state push, TLV with ascii-digit values: 0x50 = STA toggle, 0x53 = AP toggle. Published in update().
    uint16_t p = 0;
    while (p + 2 <= pll) {
      const uint8_t tag = pl[p], l = pl[p + 1];
      if (p + 2 + l > pll)
        break;
      if (l >= 1 && tag == 0x50)
        this->h310d_sta_ = static_cast<char>(pl[p + 2]);
      else if (l >= 1 && tag == 0x53)
        this->h310d_ap_ = static_cast<char>(pl[p + 2]);
      p += 2 + l;
    }
    return;
  }
  if (cmd == 0x02 && pll >= 14 && pl[0] == 0x39 && pl[1] == 12) {
    // Status for the paired remote: [1][our heartbeat WiFi byte echoed][busy][power%]
    // [real temp][real fan][set temp][set fan], int16 LE. The heating mode is not part of it.
    const uint8_t *v = pl + 2;
    this->h310d_st_busy_ = v[2] != 0;
    this->h310d_st_power_ = v[3];
    this->h310d_st_real_temp_ = static_cast<int16_t>(v[4] | (v[5] << 8));
    this->h310d_st_real_fan_ = static_cast<int16_t>(v[6] | (v[7] << 8));
    this->h310d_st_set_temp_ = static_cast<int16_t>(v[8] | (v[9] << 8));
    this->h310d_st_set_fan_ = static_cast<int16_t>(v[10] | (v[11] << 8));
    this->h310d_st_fresh_ = true;
  }
  // Other frames (idle pings, ACKs of our commands, the MCU's requests to a remote) need no action.
}

void Aixun::h310d_loop_(uint32_t now) {
  // Read everything waiting into the frame buffer (cheap, no publishing here).
  this->read_available([this](uint8_t c) {
    if (this->h310d_len_ < sizeof(this->h310d_buf_))
      this->h310d_buf_[this->h310d_len_++] = c;
    else
      this->h310d_len_ = 0;  // overflow: drop and resync
  });
  // Extract complete, CRC-valid frames; resync by dropping a byte on a bad length/crc.
  uint16_t i = 0;
  while (this->h310d_len_ - i >= 4) {
    const uint16_t total = this->h310d_buf_[i + 2] | (this->h310d_buf_[i + 3] << 8);
    if (total < 8 || total > 64) {
      i++;
      continue;
    }
    if (this->h310d_len_ - i < total)
      break;  // wait for the rest
    if (crc8_(&this->h310d_buf_[i], total - 1) == this->h310d_buf_[i + total - 1]) {
      this->h310d_dispatch_(&this->h310d_buf_[i], total);
      i += total;
    } else {
      i++;
    }
  }
  if (i > 0) {
    this->h310d_len_ -= i;
    memmove(this->h310d_buf_, this->h310d_buf_ + i, this->h310d_len_);
  }

  if (!this->h310d_connected_ && now > 5000) {
    this->h310d_connected_ = true;
    this->h310d_send_connect_();
  }
  // Refresh device info every 30 s.
  if (now - this->h310d_last_info_ > 30000) {
    this->h310d_last_info_ = now;
    this->h310d_send_(0x01, nullptr, 0);
  }
  // Status heartbeat every 2 s -> lights the panel's WiFi icon + signal bars, BLE off.
  if (now - this->h310d_last_hb_ > 2000) {
    this->h310d_last_hb_ = now;
    const uint32_t up = now / 1000;
    bool conn = false;
    int rssi = 0;
#ifdef USE_WIFI
    if (wifi::global_wifi_component != nullptr) {
      conn = wifi::global_wifi_component->is_connected();
      if (conn)
        rssi = wifi::global_wifi_component->wifi_rssi();
    }
#endif
    const uint8_t bars = !conn ? 0 : (rssi >= -55 ? 4 : rssi >= -65 ? 3 : rssi >= -75 ? 2 : 1);
    const uint8_t pl[8] = {static_cast<uint8_t>(up),        static_cast<uint8_t>(up >> 8),
                           static_cast<uint8_t>(up >> 16),  static_cast<uint8_t>(up >> 24),
                           static_cast<uint8_t>(conn ? 3 : 0), bars, 0 /* BLE off */, 1};
    this->h310d_send_(0x20, pl, 8);
  }
}

void Aixun::h310d_update_() {
#ifdef USE_TEXT_SENSOR
  if (this->h310d_info_dirty_) {
    this->h310d_info_dirty_ = false;
    if (this->device_model_ != nullptr && !this->h310d_model_.empty())
      this->device_model_->publish_state(this->h310d_model_);
    if (this->serial_number_ != nullptr && !this->h310d_serial_.empty())
      this->serial_number_->publish_state(this->h310d_serial_);
    if (this->firmware_version_ != nullptr && !this->h310d_fw_.empty())
      this->firmware_version_->publish_state(this->h310d_fw_);
  }
#endif
  if (this->h310d_st_fresh_) {
    this->h310d_st_fresh_ = false;
#ifdef USE_SENSOR
    const float vals[3] = {static_cast<float>(this->h310d_st_real_temp_),
                           static_cast<float>(this->h310d_st_real_fan_),
                           static_cast<float>(this->h310d_st_power_)};
    for (int k = 0; k < 3; k++) {
      if (this->h310d_sensors_[k] != nullptr && this->h310d_sensors_[k]->get_raw_state() != vals[k])
        this->h310d_sensors_[k]->publish_state(vals[k]);
    }
#endif
#ifdef USE_NUMBER
    // Real setpoints, so presets and changes on the panel show up too.
    if (this->temp_number_ != nullptr && this->temp_number_->state != this->h310d_st_set_temp_)
      this->temp_number_->publish_state(this->h310d_st_set_temp_);
    if (this->fan_number_ != nullptr && this->fan_number_->state != this->h310d_st_set_fan_)
      this->fan_number_->publish_state(this->h310d_st_set_fan_);
#endif
#ifdef USE_TEXT_SENSOR
    if (this->heater_status_ != nullptr) {
      const char *st = !this->h310d_st_busy_ ? "Standby" : (this->h310d_st_power_ > 0 ? "Heating" : "Cooling");
      if (this->heater_status_->state != st)
        this->heater_status_->publish_state(st);
    }
#endif
  }
#ifdef USE_BINARY_SENSOR
  // STA mode = TLV 0x50 value '2' (on) / '1' (off); AP mode = TLV 0x53 value '4' (on) / '1' (off).
  if (this->sta_bsensor_ != nullptr && this->h310d_sta_ != 0)
    this->sta_bsensor_->publish_state(this->h310d_sta_ == '2');
  if (this->ap_bsensor_ != nullptr && this->h310d_ap_ != 0)
    this->ap_bsensor_->publish_state(this->h310d_ap_ == '4');
#endif
}

void Aixun::dump_config() {
  ESP_LOGCONFIG(TAG, "Aixun:");
  LOG_UPDATE_INTERVAL(this);
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "Device model", this->device_model_);
  LOG_TEXT_SENSOR("  ", "Firmware version", this->firmware_version_);
#endif
#ifdef USE_SENSOR
  static constexpr const char *OUTPUT_NAMES[3] = {"PD+QC", "Boot", "DC"};
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++)
      LOG_SENSOR("  ", OUTPUT_NAMES[i], this->sensors_[i][j]);
  }
#endif
}

}  // namespace esphome::aixun

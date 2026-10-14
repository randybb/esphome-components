#include "aixun.h"

#include "esphome/core/log.h"

#include <cstdio>
#include <cstring>

namespace esphome::aixun {

ESPHOME_LOG_TAG(TAG, "aixun");
static const char *const OUTPUT_NAMES[3] = {"PD+QC", "Boot", "DC"};

void Aixun::loop() {
  uint32_t now = millis();
  bool connected = this->parent_->is_connected();
  // Read again after a replug, the firmware may have been updated meanwhile.
  if (!connected)
    this->have_version_ = false;
  if (connected && !this->have_version_ && (this->device_model_ != nullptr || this->firmware_version_ != nullptr) &&
      now - this->last_version_request_ > 2000) {
    this->send_("JC_version");
    this->last_version_request_ = now;
  }
  // (Re)open whenever the stream stops: USB just enumerated, or the supply was power cycled.
  if (this->p2408_ && connected && now - this->last_status_ > 2000 && now - this->last_open_ > 2000) {
    if (this->opened_)
      ESP_LOGW(TAG, "No status, reopening");
    this->send_("JCXX_open");
    this->send_("JC_Read_Set");
    this->last_open_ = now;
    this->opened_ = true;
  }

  uint8_t c;
  while (this->available() && this->read_byte(&c)) {
    this->last_byte_ = now;
    if (c == '\n') {
      this->line_[this->line_len_] = '\0';
      this->handle_line_();
      this->line_len_ = 0;
    } else if (c != '\r' && this->line_len_ < sizeof(this->line_) - 1) {
      this->line_[this->line_len_++] = c;
    }
  }
  // The JC_version reply has no line end, it is complete once the line goes quiet.
  if (this->line_len_ > 0 && now - this->last_byte_ > 50) {
    this->line_[this->line_len_] = '\0';
    this->handle_line_();
    this->line_len_ = 0;
  }
}

void Aixun::handle_line_() {
  // JC_M_P2408_version1.27: no line end, so on a P2408 the next status line follows on the same line.
  if (char *v = strstr(this->line_, "_version")) {
    char *model = this->line_;
    for (char *p = this->line_; p < v; p++) {
      if (strncmp(p, "JC_", 3) == 0)
        model = p;
    }
    *v = '\0';
    char *ver = v + strlen("_version");
    ver[strcspn(ver, "{")] = '\0';
    ESP_LOGI(TAG, "Model %s, firmware %s", model, ver);
    if (this->device_model_ != nullptr)
      this->device_model_->publish_state(model);
    if (this->firmware_version_ != nullptr)
      this->firmware_version_->publish_state(ver);
    this->have_version_ = true;
    return;
  }
  // A status line can be cut short and continued by the next one on the same line, the last '{' starts the full one.
  if (const char *p = strrchr(this->line_, '{')) {
    int work, channel, o1, o2, o3;
    float v[6];
    if (sscanf(p, "{work:%d,channel:%d,out:%d,%d,%d,V:%f,I:%f,V:%f,I:%f,V:%f,I:%f}", &work, &channel, &o1, &o2, &o3,
               &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 11) {
      for (int i = 0; i < 3; i++) {
        this->vi_[i][0] = v[2 * i];
        this->vi_[i][1] = v[2 * i + 1];
      }
      this->fresh_ = true;
      this->last_status_ = millis();
    }
    return;
  }
  // JC_Set:DC2.500,BATT3.000,DCCV,BATTCV
  if (const char *p = strstr(this->line_, "JC_Set:")) {
    float dc, batt;
    if (sscanf(p, "JC_Set:DC%f,BATT%f", &dc, &batt) == 2) {
      ESP_LOGD(TAG, "Current limits: DC %.3f A, Boot %.3f A", dc, batt);
      this->batt_limit_ = batt;
      if (this->dc_limit_number_ != nullptr)
        this->dc_limit_number_->publish_state(dc);
    }
    return;
  }
  if (this->line_len_ > 0)
    ESP_LOGV(TAG, "Other: %s", this->line_);
}

void Aixun::update() {
  for (int i = 0; i < 3; i++) {
    float v = this->fresh_ ? this->vi_[i][0] : NAN;
    float a = this->fresh_ ? this->vi_[i][1] : NAN;
    float values[3] = {v, a, v * a};
    for (int j = 0; j < 3; j++) {
      if (this->sensors_[i][j] != nullptr)
        this->sensors_[i][j]->publish_state(values[j]);
    }
  }
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

void Aixun::dump_config() {
  ESP_LOGCONFIG(TAG, "Aixun:");
  LOG_UPDATE_INTERVAL(this);
  LOG_TEXT_SENSOR("  ", "Device model", this->device_model_);
  LOG_TEXT_SENSOR("  ", "Firmware version", this->firmware_version_);
  for (int i = 0; i < 3; i++) {
    LOG_SENSOR("  ", OUTPUT_NAMES[i], this->sensors_[i][0]);
    LOG_SENSOR("  ", OUTPUT_NAMES[i], this->sensors_[i][1]);
    LOG_SENSOR("  ", OUTPUT_NAMES[i], this->sensors_[i][2]);
  }
}

}  // namespace esphome::aixun

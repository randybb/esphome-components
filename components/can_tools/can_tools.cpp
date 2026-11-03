#include "can_tools.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

#include <algorithm>

namespace esphome::can_tools {

static const char *const TAG = "can_tools";

static constexpr uint8_t NEGATIVE_RESPONSE = 0x7F;
static constexpr uint8_t RESPONSE_PENDING = 0x78;
static constexpr uint8_t POSITIVE_RESPONSE_OFFSET = 0x40;
// longer payloads are cut in the log
static constexpr size_t HEX_SIZE = format_hex_pretty_size(32);

void CanTools::setup() {
  this->canbus_->add_callback([this](uint32_t can_id, bool extended_id, bool rtr, const Payload &frame) {
    this->on_frame_(can_id, extended_id, rtr, frame);
  });
}

void CanTools::update() {
  const uint32_t now = App.get_loop_component_start_time();
  if (this->active_ != nullptr) {
    if (now - this->active_since_ < this->active_->timeout_)
      return;
    this->finish_("timeout");
  }
  // round robin, so a short interval does not starve the others
  for (size_t i = 0; i < this->requests_.size(); i++) {
    auto *req = this->requests_[(this->next_ + i) % this->requests_.size()];
    if (req->sent_once_ && now - req->last_sent_ < req->interval_)
      continue;
    this->next_ = (this->next_ + i + 1) % this->requests_.size();
    req->last_sent_ = now;
    req->sent_once_ = true;
    this->active_ = req;
    this->active_since_ = now;
    this->rx_.clear();
    this->rx_len_ = 0;
    uint8_t frame[8] = {static_cast<uint8_t>(req->data_.size())};
    std::copy(req->data_.begin(), req->data_.end(), frame + 1);
    this->send_(req->can_id_, frame, sizeof(frame));
    return;
  }
}

void CanTools::on_frame_(uint32_t can_id, bool extended_id, bool rtr, const Payload &frame) {
  auto *req = this->active_;
  if (req == nullptr || extended_id || rtr || can_id != req->response_can_id_ || frame.empty())
    return;
  switch (frame[0] >> 4) {
    case 0: {  // single frame
      size_t len = frame[0] & 0x0F;
      if (len == 0 || len >= frame.size())
        return;
      this->rx_.assign(frame.begin() + 1, frame.begin() + 1 + len);
      break;
    }
    case 1: {  // first frame: ask for the rest without block size or separation time limits
      if (frame.size() < 8)
        return;
      this->rx_len_ = ((frame[0] & 0x0F) << 8) | frame[1];
      this->rx_.assign(frame.begin() + 2, frame.end());
      this->rx_seq_ = 1;
      const uint8_t flow_control[8] = {0x30};
      this->send_(req->can_id_, flow_control, sizeof(flow_control));
      return;
    }
    case 2: {  // consecutive frame
      if (this->rx_.size() >= this->rx_len_)
        return;
      if ((frame[0] & 0x0F) != this->rx_seq_) {
        this->finish_("consecutive frame out of sequence");
        return;
      }
      this->rx_seq_ = (this->rx_seq_ + 1) & 0x0F;
      size_t n = std::min(frame.size() - 1, this->rx_len_ - this->rx_.size());
      this->rx_.insert(this->rx_.end(), frame.begin() + 1, frame.begin() + 1 + n);
      if (this->rx_.size() < this->rx_len_)
        return;
      break;
    }
    default:
      return;
  }

  char hex[HEX_SIZE];
  if (this->rx_[0] == NEGATIVE_RESPONSE) {
    if (this->rx_.size() >= 3 && this->rx_[2] == RESPONSE_PENDING) {
      this->active_since_ = App.get_loop_component_start_time();  // the ECU needs more time
      return;
    }
    ESP_LOGW(TAG, "0x%03" PRIX32 " negative response %s", req->can_id_, format_hex_pretty_to(hex, this->rx_));
    this->finish_(nullptr);
    return;
  }
  if (this->rx_[0] != req->data_[0] + POSITIVE_RESPONSE_OFFSET) {
    ESP_LOGV(TAG, "0x%03" PRIX32 " ignoring %s", req->can_id_, format_hex_pretty_to(hex, this->rx_));
    return;
  }
  ESP_LOGV(TAG, "0x%03" PRIX32 " response %s", req->can_id_, format_hex_pretty_to(hex, this->rx_));
  this->active_ = nullptr;
  req->callbacks_.call(this->rx_);
}

void CanTools::send_(uint32_t can_id, const uint8_t *data, size_t len) {
  if (this->canbus_->send_data(can_id, false, false, data, len) != canbus::ERROR_OK)
    ESP_LOGW(TAG, "0x%03" PRIX32 " send failed", can_id);
}

void CanTools::finish_(const char *reason) {
  char hex[HEX_SIZE];
  if (reason != nullptr)
    ESP_LOGD(TAG, "0x%03" PRIX32 " %s: %s", this->active_->can_id_, format_hex_pretty_to(hex, this->active_->data_),
             reason);
  this->active_ = nullptr;
}

void CanTools::dump_config() {
  ESP_LOGCONFIG(TAG, "CAN tools:");
  LOG_UPDATE_INTERVAL(this);
  char hex[HEX_SIZE];
  for (auto *req : this->requests_) {
    ESP_LOGCONFIG(TAG, "  Request 0x%03" PRIX32 " -> 0x%03" PRIX32 ": %s every %" PRIu32 " ms", req->can_id_,
                  req->response_can_id_, format_hex_pretty_to(hex, req->data_), req->interval_);
  }
}

}  // namespace esphome::can_tools

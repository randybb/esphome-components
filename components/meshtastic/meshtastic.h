#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

#include "esphome/components/sx127x/sx127x.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome::meshtastic {

static constexpr uint32_t BROADCAST = 0xFFFFFFFF;

// A Meshtastic node on one channel: decrypts what the radio receives, sends text and our NodeInfo.
// It does not relay, other nodes keep doing that.
class Meshtastic : public Component, public sx127x::SX127xListener, public Parented<sx127x::SX127x> {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }
  void on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) override;

  void set_channel(const std::string &name, const std::vector<uint8_t> &key);
  void set_node_id(uint32_t node_id) { this->node_id_ = node_id; }
  void set_names(const std::string &long_name, const std::string &short_name) {
    this->long_name_ = long_name;
    this->short_name_ = short_name;
  }
  void set_hop_limit(uint8_t hop_limit) { this->hop_limit_ = hop_limit; }

  bool send_text(const std::string &text, uint32_t to = BROADCAST);
  void send_node_info(uint32_t to = BROADCAST, bool want_response = false);
  // Long name from the node's NodeInfo, or !xxxxxxxx
  std::string node_name(uint32_t node) const;
  uint32_t get_node_id() const { return this->node_id_; }

  Trigger<uint32_t, uint32_t, std::string, std::string, float, float> *get_message_trigger() {
    return &this->message_trigger_;
  }

 protected:
  bool send_(uint32_t to, uint8_t portnum, const std::string &payload, bool want_response = false);
  void crypt_(uint32_t from, uint32_t id, uint8_t *data, size_t len) const;
  bool seen_(uint32_t from, uint32_t id);

  std::string channel_;
  std::vector<uint8_t> key_;
  uint8_t hash_{0};
  uint32_t node_id_{0};
  std::string long_name_;
  std::string short_name_;
  uint8_t hop_limit_{3};
  std::map<uint32_t, std::string> names_;
  std::vector<uint32_t> asked_;  // nodes asked for their NodeInfo
  std::array<uint64_t, 32> seen_ids_{};  // from << 32 | packet id, relays repeat packets
  uint8_t seen_next_{0};
  Trigger<uint32_t, uint32_t, std::string, std::string, float, float> message_trigger_;
};

}  // namespace esphome::meshtastic

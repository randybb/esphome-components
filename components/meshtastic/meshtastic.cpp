#include "meshtastic.h"

#include <algorithm>
#include <cinttypes>
#include <cstring>

#include <mbedtls/aes.h>

#include "esphome/core/alloc_helpers.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::meshtastic {

static const char *const TAG = "meshtastic";

// From meshtastic/firmware src/mesh: RadioInterface.h (header), Channels.h (defaultpsk), portnums.proto
static constexpr size_t HEADER_SIZE = 16;  // to, from, id (LE), flags, channel hash, next hop, relay node
static constexpr size_t MAX_PACKET = 255;
static constexpr uint8_t FLAGS_HOP_LIMIT_MASK = 0x07;
static constexpr uint8_t FLAGS_HOP_START_SHIFT = 5;
static constexpr uint8_t DEFAULT_PSK[16] = {0xd4, 0xf1, 0xbb, 0x3a, 0x20, 0x29, 0x07, 0x59,
                                            0xf0, 0xbc, 0xff, 0xab, 0xcf, 0x4e, 0x69, 0x01};
static constexpr uint8_t PORT_TEXT_MESSAGE = 1;
static constexpr uint8_t PORT_POSITION = 3;
static constexpr uint8_t PORT_NODEINFO = 4;
static constexpr uint8_t PORT_ROUTING = 5;
static constexpr uint8_t PORT_TELEMETRY = 67;
static constexpr uint8_t HW_MODEL_PRIVATE = 255;
static constexpr uint8_t ROLE_CLIENT_MUTE = 1;  // we don't relay

static uint32_t get_le32(const uint8_t *p) {
  return uint32_t(p[0]) | uint32_t(p[1]) << 8 | uint32_t(p[2]) << 16 | uint32_t(p[3]) << 24;
}
static void put_le32(uint8_t *p, uint32_t v) {
  for (int i = 0; i < 4; i++)
    p[i] = v >> (8 * i);
}

// Minimal protobuf: walks the top level fields of a message.
struct Field {
  uint32_t number;
  uint64_t value;     // varint and fixed
  const uint8_t *data;  // length delimited
  size_t len;
};
static bool read_varint(const uint8_t *&p, const uint8_t *end, uint64_t &out) {
  out = 0;
  for (int shift = 0; p < end && shift < 64; shift += 7) {
    const uint8_t b = *p++;
    out |= uint64_t(b & 0x7F) << shift;
    if (!(b & 0x80))
      return true;
  }
  return false;
}
template<typename F> static bool for_each_field(const uint8_t *p, size_t len, F fn) {
  const uint8_t *end = p + len;
  while (p < end) {
    uint64_t tag;
    if (!read_varint(p, end, tag))
      return false;
    Field f{uint32_t(tag >> 3), 0, nullptr, 0};
    switch (tag & 7) {
      case 0:
        if (!read_varint(p, end, f.value))
          return false;
        break;
      case 1:
        if (end - p < 8)
          return false;
        f.value = get_le32(p) | uint64_t(get_le32(p + 4)) << 32;
        p += 8;
        break;
      case 2: {
        uint64_t n;
        if (!read_varint(p, end, n) || n > uint64_t(end - p))
          return false;
        f.data = p;
        f.len = n;
        p += n;
        break;
      }
      case 5:
        if (end - p < 4)
          return false;
        f.value = get_le32(p);
        p += 4;
        break;
      default:
        return false;
    }
    fn(f);
  }
  return true;
}
static void put_varint(std::string &out, uint64_t v) {
  do {
    uint8_t b = v & 0x7F;
    v >>= 7;
    out += char(v ? b | 0x80 : b);
  } while (v);
}
static void put_bytes(std::string &out, uint32_t number, const std::string &value) {
  put_varint(out, number << 3 | 2);
  put_varint(out, value.size());
  out += value;
}

void Meshtastic::set_channel(const std::string &name, const std::vector<uint8_t> &key) {
  this->channel_ = name;
  this->key_ = key;
  if (key.size() == 1) {
    // Short PSK index as in Channels::getKey: 0 is no encryption, n is the default key with the last byte + n - 1
    if (key[0] == 0) {
      this->key_.clear();
    } else {
      this->key_.assign(DEFAULT_PSK, DEFAULT_PSK + sizeof(DEFAULT_PSK));
      this->key_.back() += key[0] - 1;
    }
  }
  uint8_t h = 0;
  for (char c : name)
    h ^= uint8_t(c);
  for (uint8_t b : this->key_)
    h ^= b;
  this->hash_ = h;
}

void Meshtastic::setup() {
  if (this->node_id_ == 0) {
    // as the firmware does: the last four bytes of the MAC
    uint8_t mac[6];
    get_mac_address_raw(mac);
    this->node_id_ = uint32_t(mac[2]) << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];
  }
  this->parent_->register_listener(this);
  if (!this->long_name_.empty()) {
    // announce ourselves after the radio started, then every 3 hours like the firmware's default
    this->set_timeout(30000, [this]() { this->send_node_info(); });
    this->set_interval(3 * 60 * 60 * 1000, [this]() { this->send_node_info(); });
  }
}

void Meshtastic::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Meshtastic:\n"
                "  Node: !%08" PRIx32 " %s\n"
                "  Channel: %s (hash 0x%02X, %s)\n"
                "  Hop limit: %u",
                this->node_id_, this->long_name_.c_str(), this->channel_.c_str(), this->hash_,
                this->key_.empty() ? "no encryption" : this->key_.size() == 16 ? "AES-128" : "AES-256",
                this->hop_limit_);
}

// AES-CTR, the nonce is the packet id (64 bit LE) and the sender (32 bit LE). Encrypting and decrypting are the same.
void Meshtastic::crypt_(uint32_t from, uint32_t id, uint8_t *data, size_t len) const {
  if (this->key_.empty())
    return;
  uint8_t nonce[16]{};
  put_le32(nonce, id);
  put_le32(nonce + 8, from);
  uint8_t stream[16];
  size_t offset = 0;
  mbedtls_aes_context aes;
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, this->key_.data(), this->key_.size() * 8);
  mbedtls_aes_crypt_ctr(&aes, len, &offset, nonce, stream, data, data);
  mbedtls_aes_free(&aes);
}

bool Meshtastic::seen_(uint32_t from, uint32_t id) {
  const uint64_t key = uint64_t(from) << 32 | id;
  if (std::find(this->seen_ids_.begin(), this->seen_ids_.end(), key) != this->seen_ids_.end())
    return true;
  this->seen_ids_[this->seen_next_] = key;
  this->seen_next_ = (this->seen_next_ + 1) % this->seen_ids_.size();
  return false;
}

std::string Meshtastic::node_name(uint32_t node) const {
  if (node == BROADCAST)
    return "all";
  auto it = this->names_.find(node);
  if (it != this->names_.end())
    return it->second;
  return str_sprintf("!%08" PRIx32, node);
}

void Meshtastic::on_packet(const std::vector<uint8_t> &packet, float rssi, float snr) {
  if (packet.size() < HEADER_SIZE)
    return;
  ESP_LOGV(TAG, "Packet %s", format_hex_pretty(packet).c_str());
  const uint32_t to = get_le32(&packet[0]);
  const uint32_t from = get_le32(&packet[4]);
  const uint32_t id = get_le32(&packet[8]);
  const uint8_t hops_left = packet[12] & FLAGS_HOP_LIMIT_MASK;
  const uint8_t hop_start = packet[12] >> FLAGS_HOP_START_SHIFT;
  if (from == this->node_id_ || this->seen_(from, id)) {
    ESP_LOGV(TAG, "Repeat of !%08" PRIx32 " #%08" PRIx32 ", %.0f dBm", from, id, rssi);
    return;
  }
  if (packet[13] != this->hash_) {
    ESP_LOGD(TAG, "!%08" PRIx32 " -> %s on another channel (hash 0x%02X), %.0f dBm", from, this->node_name(to).c_str(),
             packet[13], rssi);
    return;
  }
  std::vector<uint8_t> data(packet.begin() + HEADER_SIZE, packet.end());
  this->crypt_(from, id, data.data(), data.size());

  uint32_t portnum = 0;
  bool want_response = false;
  const uint8_t *payload = nullptr;
  size_t payload_len = 0;
  const bool ok = for_each_field(data.data(), data.size(), [&](const Field &f) {
    if (f.number == 1)
      portnum = f.value;
    else if (f.number == 3)
      want_response = f.value;
    else if (f.number == 2 && f.data != nullptr) {
      payload = f.data;
      payload_len = f.len;
    }
  });
  if (!ok || portnum == 0) {
    // PKI direct messages and AEAD channels are not decoded
    ESP_LOGD(TAG, "!%08" PRIx32 " -> %s: undecodable %u B, %.0f dBm", from, this->node_name(to).c_str(), data.size(),
             rssi);
    return;
  }
  const std::string from_name = this->node_name(from);
  // ponytail: one request per new node, a busy mesh after boot means a burst of them
  if (!this->long_name_.empty() && portnum != PORT_NODEINFO && this->asked_.size() < 64 &&
      this->names_.find(from) == this->names_.end() &&
      std::find(this->asked_.begin(), this->asked_.end(), from) == this->asked_.end()) {
    // ask once for the name, the node answers with its NodeInfo
    this->asked_.push_back(from);
    this->send_node_info(from, true);
  }
  const unsigned hops = hop_start >= hops_left ? hop_start - hops_left : 0;

  switch (portnum) {
    case PORT_TEXT_MESSAGE: {
      std::string text(reinterpret_cast<const char *>(payload), payload_len);
      ESP_LOGI(TAG, "%s -> %s (%u hops, %.0f dBm, SNR %.1f): %s", from_name.c_str(), this->node_name(to).c_str(), hops,
               rssi, snr, text.c_str());
      this->message_trigger_.trigger(from, to, from_name, text, rssi, snr);
      break;
    }
    case PORT_NODEINFO: {
      std::string long_name, short_name;
      for_each_field(payload, payload_len, [&](const Field &f) {
        if (f.number == 2 && f.data != nullptr)
          long_name.assign(reinterpret_cast<const char *>(f.data), f.len);
        else if (f.number == 3 && f.data != nullptr)
          short_name.assign(reinterpret_cast<const char *>(f.data), f.len);
      });
      ESP_LOGI(TAG, "!%08" PRIx32 " is %s (%s), %u hops, %.0f dBm", from, long_name.c_str(), short_name.c_str(), hops,
               rssi);
      if (!long_name.empty()) {
        // ponytail: drops an arbitrary node when full, an LRU if meshes get bigger
        if (this->names_.size() >= 64 && this->names_.find(from) == this->names_.end())
          this->names_.erase(this->names_.begin());
        this->names_[from] = long_name;
      }
      if (want_response && to == this->node_id_ && !this->long_name_.empty())
        this->send_node_info(from, false);
      break;
    }
    case PORT_POSITION:
    case PORT_ROUTING:
    case PORT_TELEMETRY:
    default:
      ESP_LOGD(TAG, "%s -> %s: port %" PRIu32 ", %u B, %u hops, %.0f dBm", from_name.c_str(),
               this->node_name(to).c_str(), portnum, payload_len, hops, rssi);
      break;
  }
}

bool Meshtastic::send_(uint32_t to, uint8_t portnum, const std::string &payload, bool want_response) {
  std::string data;
  put_varint(data, 1 << 3 | 0);
  put_varint(data, portnum);
  put_bytes(data, 2, payload);
  if (want_response) {
    put_varint(data, 3 << 3 | 0);
    put_varint(data, 1);
  }
  if (HEADER_SIZE + data.size() > MAX_PACKET) {
    ESP_LOGW(TAG, "Payload too long (%u B)", payload.size());
    return false;
  }
  const uint32_t id = random_uint32();
  this->seen_(this->node_id_, id);
  std::vector<uint8_t> packet(HEADER_SIZE + data.size());
  put_le32(&packet[0], to);
  put_le32(&packet[4], this->node_id_);
  put_le32(&packet[8], id);
  packet[12] = this->hop_limit_ | this->hop_limit_ << FLAGS_HOP_START_SHIFT;
  packet[13] = this->hash_;
  packet[14] = 0;                       // next hop: unknown, flood
  packet[15] = this->node_id_ & 0xFF;  // relay node: us
  memcpy(&packet[HEADER_SIZE], data.data(), data.size());
  this->crypt_(this->node_id_, id, &packet[HEADER_SIZE], data.size());
  // ponytail: blocks for the airtime (~0.5 s at LongFast), no queue and no duty cycle tracking
  if (this->parent_->transmit_packet(packet) != sx127x::SX127xError::NONE) {
    ESP_LOGW(TAG, "Transmit failed");
    return false;
  }
  return true;
}

bool Meshtastic::send_text(const std::string &text, uint32_t to) {
  ESP_LOGI(TAG, "Sending to %s: %s", this->node_name(to).c_str(), text.c_str());
  return this->send_(to, PORT_TEXT_MESSAGE, text);
}

void Meshtastic::send_node_info(uint32_t to, bool want_response) {
  std::string user;
  put_bytes(user, 1, str_sprintf("!%08" PRIx32, this->node_id_));
  put_bytes(user, 2, this->long_name_);
  put_bytes(user, 3, this->short_name_);
  put_varint(user, 5 << 3 | 0);
  put_varint(user, HW_MODEL_PRIVATE);
  put_varint(user, 7 << 3 | 0);
  put_varint(user, ROLE_CLIENT_MUTE);
  ESP_LOGD(TAG, "Sending NodeInfo to %s%s", this->node_name(to).c_str(), want_response ? ", asking for theirs" : "");
  this->send_(to, PORT_NODEINFO, user, want_response);
}

}  // namespace esphome::meshtastic

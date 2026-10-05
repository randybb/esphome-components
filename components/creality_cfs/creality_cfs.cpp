#include "creality_cfs.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "esphome/core/log.h"
#include "mbedtls/aes.h"

namespace esphome::creality_cfs {

static const char *const TAG = "creality_cfs";

// Keys of the Creality firmware, as published by the community (DnG-Crafts/K2-RFID)
static const uint8_t KEY_UID[16] = {'q', '3', 'b', 'u', '^', 't', '1', 'n', 'q', 'f', 'Z', '(', 'p', 'f', '$', '1'};
static const uint8_t KEY_DATA[16] = {'H', '@', 'C', 'F', 'k', 'R', 'n', 'z', '@', 'K', 'A', 't', 'B', 'J', 'p', '2'};
static const uint8_t KEY_FACTORY[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
// Access bits of a factory sector trailer (bytes 6-8)
static const uint8_t ACCESS_FACTORY[3] = {0xFF, 0x07, 0x80};

static constexpr uint8_t SECTOR_1_BLOCK = 4;  // blocks 4-6, AES encrypted
static constexpr uint8_t SECTOR_1_TRAILER = 7;
static constexpr uint8_t SECTOR_2_BLOCK = 8;  // blocks 8-10, plain, factory key
static constexpr size_t RECORD_SIZE = 48;
static constexpr Field FIELD_LENGTH = {24, 4};  // hex, meters

// MIFARE Classic 1K, 4K and Mini, with or without the ISO 14443-4 bit some clones set
static bool is_mifare_classic(uint8_t sak) {
  sak &= ~0x80;
  return sak == 0x08 || sak == 0x18 || sak == 0x09;
}

static void aes_ecb(const uint8_t *key, bool encrypt, const uint8_t *in, uint8_t *out, size_t length) {
  mbedtls_aes_context ctx;
  mbedtls_aes_init(&ctx);
  if (encrypt)
    mbedtls_aes_setkey_enc(&ctx, key, 128);
  else
    mbedtls_aes_setkey_dec(&ctx, key, 128);
  for (size_t i = 0; i + 16 <= length; i += 16)
    mbedtls_aes_crypt_ecb(&ctx, encrypt ? MBEDTLS_AES_ENCRYPT : MBEDTLS_AES_DECRYPT, in + i, out + i);
  mbedtls_aes_free(&ctx);
}

void CrealityCfsComponent::dump_config() { ESP_LOGCONFIG(TAG, "Creality CFS"); }

void CrealityCfsComponent::write(const std::string &uid, const std::string &data) {
  if (uid.empty() || data.size() < 34 || data.size() > 2 * RECORD_SIZE) {
    ESP_LOGW(TAG, "Write: need a UID and a record of 34 to 96 characters");
    return;
  }
  this->pending_uid_ = uid;
  this->pending_data_ = data;
  ESP_LOGD(TAG, "Write to %s queued for its next poll", uid.c_str());
}

void CrealityCfsComponent::on_iso14443a(const std::string &uid, const std::vector<uint8_t> &uid_bytes,
                                        uint8_t sak) {
  if (uid_bytes.size() != 4 || !is_mifare_classic(sak))
    return;
  if (uid != this->uid_) {
    this->on_tag_removed(this->uid_);
    this->uid_ = uid;
    this->read_done_ = false;
    this->default_key_ = false;
    // Key A and B of sector 1: AES-ECB of the UID repeated to 16 bytes, its first 6 bytes
    uint8_t in[16], out[16];
    for (size_t i = 0; i < 16; i++)
      in[i] = uid_bytes[i % 4];
    aes_ecb(KEY_UID, true, in, out, 16);
    std::copy(out, out + 6, this->key_);
  }
  const bool pending = this->pending_uid_ == uid;
  if (this->read_done_ && !pending)
    return;

  const bool blank = this->default_key_;
  if (!this->reader_->mifare_authenticate(SECTOR_1_BLOCK, blank ? KEY_FACTORY : this->key_)) {
    // A failed authentication drops the selection: the other key on the next poll
    this->default_key_ = !this->default_key_;
    return;
  }
  if (pending) {
    const bool ok = this->write_(blank);
    ESP_LOGI(TAG, "Write to %s %s", uid.c_str(), ok ? "done" : "failed");
    this->pending_uid_.clear();
    this->pending_data_.clear();
    this->read_done_ = false;
    this->default_key_ = false;  // after a write the tag has the derived key
    return;                      // read back on the next poll
  }

  std::string record;
  if (!this->read_(blank, record))
    return;
  this->read_done_ = true;
  if (record.empty()) {
    ESP_LOGD(TAG, "Blank or foreign MIFARE Classic tag %s", uid.c_str());
    return;
  }
  ESP_LOGD(TAG, "Creality tag %s: %s", uid.c_str(), record.c_str());
  this->published_ = true;
  this->publish_(record);
  this->creality_callback_.call(uid, record);
}

void CrealityCfsComponent::on_tag_removed(const std::string &uid) {
  if (uid.empty() || uid != this->uid_)
    return;
  this->uid_.clear();
  this->read_done_ = false;
  if (this->published_) {
    this->published_ = false;
    this->publish_("");
    this->creality_callback_.call(uid, "");
  }
}

// The record as text, empty when it is blank or not a Creality one
bool CrealityCfsComponent::read_(bool blank, std::string &record) {
  uint8_t raw[2 * RECORD_SIZE] = {};
  for (uint8_t i = 0; i < 3; i++) {
    if (!this->reader_->mifare_read(SECTOR_1_BLOCK + i, raw + 16 * i))
      return false;
  }
  // A blank tag (factory key) holds plain data, a programmed one encrypted
  if (!blank)
    aes_ecb(KEY_DATA, false, raw, raw, RECORD_SIZE);
  // Sector 2 is optional, plain with the factory key (nested authentication)
  bool sector_2 = this->reader_->mifare_authenticate(SECTOR_2_BLOCK, KEY_FACTORY);
  for (uint8_t i = 0; sector_2 && i < 3; i++)
    sector_2 = this->reader_->mifare_read(SECTOR_2_BLOCK + i, raw + RECORD_SIZE + 16 * i);

  record.assign(reinterpret_cast<const char *>(raw), sector_2 ? 2 * RECORD_SIZE : RECORD_SIZE);
  record.erase(std::find(record.begin(), record.end(), '\0'), record.end());
  record.erase(record.find_last_not_of(' ') + 1);
  const bool text = std::all_of(record.begin(), record.end(), [](char c) { return c >= 0x20 && c < 0x7F; });
  if (!text || record.size() < 34 || record.find_first_not_of('0') == std::string::npos)
    record.clear();
  return true;
}

bool CrealityCfsComponent::write_(bool blank) {
  std::string data = this->pending_data_;
  data.resize(data.size() > RECORD_SIZE ? 2 * RECORD_SIZE : RECORD_SIZE, ' ');
  uint8_t block[16];
  uint8_t encrypted[RECORD_SIZE];
  aes_ecb(KEY_DATA, true, reinterpret_cast<const uint8_t *>(data.data()), encrypted, RECORD_SIZE);
  for (uint8_t i = 0; i < 3; i++) {
    if (!this->reader_->mifare_write(SECTOR_1_BLOCK + i, encrypted + 16 * i))
      return false;
  }
  if (blank) {
    // Key A and B of sector 1 from the UID, as the printer expects; the access bits stay. Only a
    // factory trailer is touched, a wrong one can lock the sector for good.
    if (!this->reader_->mifare_read(SECTOR_1_TRAILER, block) || memcmp(block + 6, ACCESS_FACTORY, 3) != 0) {
      ESP_LOGW(TAG, "Sector 1 trailer not as from the factory, keys not set");
      return false;
    }
    std::copy(this->key_, this->key_ + 6, block);
    std::copy(this->key_, this->key_ + 6, block + 10);
    if (!this->reader_->mifare_write(SECTOR_1_TRAILER, block))
      return false;
  }
  if (data.size() > RECORD_SIZE) {
    if (!this->reader_->mifare_authenticate(SECTOR_2_BLOCK, KEY_FACTORY))
      return false;
    for (uint8_t i = 0; i < 3; i++) {
      if (!this->reader_->mifare_write(SECTOR_2_BLOCK + i,
                                       reinterpret_cast<const uint8_t *>(data.data()) + RECORD_SIZE + 16 * i))
        return false;
    }
  }
  return true;
}

void CrealityCfsComponent::publish_(const std::string &record) {
  auto field = [&record](Field f) { return record.size() >= f.offset + f.length ? record.substr(f.offset, f.length) : ""; };
#ifdef USE_SENSOR
  if (this->length_sensor_ != nullptr) {
    const std::string hex = field(FIELD_LENGTH);
    char *end = nullptr;
    const long meters = hex.empty() ? 0 : strtol(hex.c_str(), &end, 16);
    this->length_sensor_->publish_state(hex.empty() || *end != '\0' ? NAN : float(meters));
  }
#endif
#ifdef USE_TEXT_SENSOR
  for (auto &f : this->text_sensors_) {
    std::string value = field(f.field);
    if (f.color && value.size() == 7)
      value = "#" + value.substr(1);  // 0RRGGBB
    f.sensor->publish_state(value);
  }
#endif
}

}  // namespace esphome::creality_cfs

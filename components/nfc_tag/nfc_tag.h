#pragma once

#include "esphome/core/component.h"

#include <string>

namespace esphome::nfc_tag {

// NFC-A Type 2 tag emulated by the nRF NFCT peripheral, serving one NDEF URI record
class NfcTag : public Component {
 public:
  void set_uri(uint8_t code, const char *uri) {
    this->uri_code_ = code;
    this->uri_ = uri;
  }
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

 protected:
  uint8_t uri_code_{0};
  std::string uri_;
  uint8_t ndef_[256];  // must outlive the emulation, the library reads it on every READ
};

}  // namespace esphome::nfc_tag

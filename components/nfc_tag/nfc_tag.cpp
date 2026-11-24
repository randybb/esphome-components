#include "nfc_tag.h"

#include "esphome/core/log.h"

#include <nfc_t2t_lib.h>
#include <nfc/ndef/uri_msg.h>

namespace esphome::nfc_tag {

static const char *const TAG = "nfc_tag";

// runs in the NFC library's interrupt context: log nothing here
static void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t data_length) {}

void NfcTag::setup() {
  uint32_t len = sizeof(this->ndef_);
  int err = nfc_ndef_uri_msg_encode(static_cast<nfc_ndef_uri_rec_id>(this->uri_code_),
                                    reinterpret_cast<const uint8_t *>(this->uri_.data()), this->uri_.size(),
                                    this->ndef_, &len);
  if (err == 0)
    err = nfc_t2t_setup(nfc_callback, nullptr);
  if (err == 0)
    err = nfc_t2t_payload_set(this->ndef_, len);
  if (err == 0)
    err = nfc_t2t_emulation_start();
  if (err != 0) {
    ESP_LOGE(TAG, "NFC T2T start failed: %d", err);
    this->mark_failed();
  }
}

void NfcTag::dump_config() {
  ESP_LOGCONFIG(TAG,
                "NFC tag (Type 2):\n"
                "  URI prefix code: 0x%02X\n"
                "  URI: %s",
                this->uri_code_, this->uri_.c_str());
}

}  // namespace esphome::nfc_tag

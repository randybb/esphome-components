#pragma once

#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/gpio.h"

#ifdef USE_SWD_FLASH
#include "esphome/components/http_request/http_request.h"
#endif

#include <cstdint>
#include <string>

namespace esphome::swd {

/// SWD probe bit-banged on two GPIOs.
///
/// Connects to an ARM Cortex-M target (MEM-AP 0), finds the SEGGER RTT control block in its RAM and
/// prints up-buffer 0 to the log line by line. The target keeps running, memory is read in the
/// background. Reconnects and looks RTT up again when the target goes away or resets.
class SWD : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_swclk_pin(InternalGPIOPin *pin) { this->swclk_ = pin; }
  void set_swdio_pin(InternalGPIOPin *pin) { this->swdio_ = pin; }
  void set_ram(uint32_t start, uint32_t size) {
    this->ram_start_ = start;
    this->ram_end_ = start + size;
  }
  void set_clock_delay(uint32_t us) { this->clock_delay_us_ = us; }

  /// Resets the target through AIRCR SYSRESETREQ, RTT is looked up again afterwards.
  void reset_target();

  bool read_word(uint32_t addr, uint32_t *value);
  bool write_word(uint32_t addr, uint32_t value);

#ifdef USE_SWD_FLASH
  void set_http_request(http_request::HttpRequestComponent *http) { this->http_ = http; }
  /// Downloads a raw binary and writes it to the target's flash at `address` (STM32F1/F4), then resets the target.
  /// Blocks until done, like HTTP OTA. `address` must start a page/sector, the image is erased lazily behind it.
  bool flash(const std::string &url, uint32_t address = 0x08000000);
#endif

 protected:
  enum class State : uint8_t { DISCONNECTED, SCANNING, RTT };

  void write_bits_(uint32_t value, uint8_t count);
  uint32_t read_bits_(uint8_t count);
  void clock_();
  void swdio_output_(bool output);
  bool transfer_(bool ap, bool read, uint8_t addr, uint32_t *data);
  bool connect_();
  bool scan_();
  bool poll_rtt_();
  void put_char_(char c);
  void disconnect_(uint32_t retry_ms);
  bool set_csw_(uint8_t size, bool increment);
  bool write_block_(uint32_t addr, const uint8_t *data, size_t len, uint8_t size);
  bool read_block_(uint32_t addr, uint8_t *data, size_t len);
#ifdef USE_SWD_FLASH
  enum class Family : uint8_t { NONE, F1, F4 };
  bool halt_after_reset_();
  bool flash_wait_(uint32_t timeout_ms);
  bool flash_erase_(uint32_t addr, uint32_t *next);
  bool flash_program_(uint32_t addr, const uint8_t *data, size_t len);
  bool flash_download_(const std::string &url, uint32_t address);

  http_request::HttpRequestComponent *http_{nullptr};
  Family family_{Family::NONE};
  uint32_t page_size_{0};  // F1
#endif

  InternalGPIOPin *swclk_{nullptr};
  InternalGPIOPin *swdio_{nullptr};
  uint32_t ram_start_{0};
  uint32_t ram_end_{0};
  uint32_t clock_delay_us_{1};

  State state_{State::DISCONNECTED};
  uint32_t next_try_ms_{0};
  bool warned_{false};
  uint32_t idcode_{0};
  uint32_t csw_{0};
  uint32_t scan_addr_{0};
  uint32_t rtt_cb_{0};
  uint32_t rtt_buffer_{0};
  uint32_t rtt_size_{0};
  std::string line_;
};

}  // namespace esphome::swd

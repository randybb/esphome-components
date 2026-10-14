#pragma once

#include "esphome/components/http_request/http_request.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"

#include <cstdint>
#include <string>

namespace esphome::stm32_bootloader {

/// Flashes an STM32 through its ROM UART bootloader (AN3155), entered with BOOT0 and NRST.
///
/// Idle otherwise: between flashes the UART belongs to whatever else uses it (e.g. packet_transport).
class STM32Bootloader : public Component, public uart::UARTDevice {
 public:
  void setup() override;
  void dump_config() override;

  void set_boot0_pin(GPIOPin *pin) { this->boot0_ = pin; }
  void set_reset_pin(GPIOPin *pin) { this->reset_ = pin; }
  void set_http_request(http_request::HttpRequestComponent *http) { this->http_ = http; }

  /// Downloads a raw binary and writes it to the target's flash at `address` (STM32F1/F4), then resets the target
  /// into its firmware. Blocks until done, like HTTP OTA. `address` must start a page/sector, the image is erased
  /// lazily behind it.
  bool flash(const std::string &url, uint32_t address = 0x08000000);

  /// Pulses NRST with BOOT0 low, the target starts its firmware.
  void reset_target();

 protected:
  enum class Family : uint8_t { NONE, F1, F4 };

  bool enter_();
  void leave_();
  bool wait_ack_(uint32_t timeout_ms);
  bool read_bytes_(uint8_t *data, size_t len, uint32_t timeout_ms);
  bool command_(uint8_t cmd);
  bool send_with_checksum_(const uint8_t *data, size_t len, uint32_t timeout_ms);
  bool send_address_(uint32_t addr);
  bool get_();
  bool get_id_(uint16_t *pid);
  bool erase_(uint32_t addr, uint32_t *next);
  bool write_(uint32_t addr, const uint8_t *data, size_t len);
  bool verify_(uint32_t addr, const uint8_t *data, size_t len);
  bool download_(const std::string &url, uint32_t address);

  GPIOPin *boot0_{nullptr};
  GPIOPin *reset_{nullptr};
  http_request::HttpRequestComponent *http_{nullptr};
  Family family_{Family::NONE};
  uint32_t page_size_{0};  // F1
  bool extended_erase_{false};
  uart::UARTParityOptions parity_{uart::UART_CONFIG_PARITY_NONE};
};

}  // namespace esphome::stm32_bootloader

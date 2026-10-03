#pragma once

#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <vector>

// Plain C++ (no ESPHome headers) so the parser can be tested on the host.
namespace esphome::openprinttag {

// One decoded CBOR value: numbers in `number`, text and byte strings in `bytes`.
// Arrays and maps (e.g. tags) are skipped and stay empty.
struct OptValue {
  bool is_number{false};
  double number{0};
  std::string bytes;
};

using OptRegion = std::map<uint32_t, OptValue>;

struct OpenPrintTag {
  OptRegion main;
  OptRegion aux;
  std::vector<uint8_t> payload;  // the whole NDEF record payload, for decoding elsewhere
  size_t payload_offset{0};      // of the payload in the tag memory
  size_t aux_offset{0};          // of the aux region in the payload
  size_t aux_size{0};            // 0: no aux region
};

// Memory size of a Type 5 tag from its capability container, 0 if it has none.
size_t t5t_memory_size(std::span<const uint8_t> cc);

// Parses a Type 5 tag memory image (CC, NDEF TLV) holding an application/vnd.openprinttag record.
bool parse_openprinttag(std::span<const uint8_t> mem, OpenPrintTag &out);

}  // namespace esphome::openprinttag

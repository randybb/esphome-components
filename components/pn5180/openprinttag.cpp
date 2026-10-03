#include "openprinttag.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <string_view>

namespace esphome::pn5180 {

static constexpr std::string_view MIME_TYPE = "application/vnd.openprinttag";
static constexpr uint8_t TNF_MIME = 0x02;
static constexpr uint8_t TLV_NULL = 0x00;
static constexpr uint8_t TLV_NDEF = 0x03;
static constexpr uint8_t TLV_TERMINATOR = 0xFE;
static constexpr uint8_t CBOR_BREAK = 0xFF;
static constexpr uint8_t CBOR_INDEFINITE = 31;
// The spec nests one level (tags array), the limit keeps a hostile tag off the stack
static constexpr uint8_t CBOR_MAX_DEPTH = 8;

namespace {

struct Cbor {
  std::span<const uint8_t> data;
  size_t pos{0};

  bool head(uint8_t &major, uint8_t &info, uint64_t &arg) {
    if (this->pos >= this->data.size())
      return false;
    const uint8_t b = this->data[this->pos++];
    major = b >> 5;
    info = b & 0x1F;
    arg = 0;
    if (info < 24 || info == CBOR_INDEFINITE) {
      arg = info < 24 ? info : 0;
      return true;
    }
    if (info > 27)
      return false;
    const size_t n = size_t{1} << (info - 24);
    if (this->pos + n > this->data.size())
      return false;
    for (size_t i = 0; i < n; i++)
      arg = (arg << 8) | this->data[this->pos++];
    return true;
  }

  bool at_break() {
    if (this->pos < this->data.size() && this->data[this->pos] == CBOR_BREAK) {
      this->pos++;
      return true;
    }
    return false;
  }

  // Decodes one item into `out` (or skips it when null)
  bool item(OptValue *out, uint8_t depth = 0) {
    if (depth > CBOR_MAX_DEPTH)
      return false;
    uint8_t major, info;
    uint64_t arg;
    if (!this->head(major, info, arg))
      return false;
    switch (major) {
      case 0:  // unsigned int
      case 1:  // negative int
        if (out != nullptr) {
          out->is_number = true;
          out->number = major == 0 ? double(arg) : -1.0 - double(arg);
        }
        return true;
      case 2:  // byte string
      case 3:  // text string
        // indefinite strings are not produced by the reference encoder
        if (info == CBOR_INDEFINITE || arg > this->data.size() - this->pos)
          return false;
        if (out != nullptr)
          out->bytes.assign(reinterpret_cast<const char *>(&this->data[this->pos]), arg);
        this->pos += arg;
        return true;
      case 4:  // array
      case 5:  // map
        for (uint64_t i = 0; info == CBOR_INDEFINITE || i < (major == 5 ? arg * 2 : arg); i++) {
          if (info == CBOR_INDEFINITE && this->at_break())
            return true;
          if (!this->item(nullptr, depth + 1))
            return false;
        }
        return true;
      case 6:  // semantic tag, the tagged value follows
        return this->item(out, depth + 1);
      default:  // 7: simple values and floats
        if (out == nullptr)
          return info != CBOR_INDEFINITE;
        if (info == 20 || info == 21) {  // false, true
          out->is_number = true;
          out->number = info - 20;
        } else if (info == 25) {
          out->is_number = true;
          const int exp = (arg >> 10) & 0x1F;
          const int mant = arg & 0x3FF;
          double v = exp == 0 ? std::ldexp(mant, -24) : exp == 31 ? NAN : std::ldexp(mant + 1024, exp - 25);
          out->number = (arg & 0x8000) ? -v : v;
        } else if (info == 26) {
          float f;
          const auto bits = uint32_t(arg);
          std::memcpy(&f, &bits, sizeof(f));
          out->is_number = true;
          out->number = f;
        } else if (info == 27) {
          std::memcpy(&out->number, &arg, sizeof(out->number));
          out->is_number = true;
        }
        return info != CBOR_INDEFINITE;
    }
  }

  // Decodes a map with integer keys; non-integer keys are skipped
  bool map(OptRegion &out) {
    uint8_t major, info;
    uint64_t arg;
    if (!this->head(major, info, arg) || major != 5)
      return false;
    for (uint64_t i = 0; info == CBOR_INDEFINITE || i < arg; i++) {
      if (info == CBOR_INDEFINITE && this->at_break())
        return true;
      OptValue key, value;
      if (!this->item(&key) || !this->item(&value))
        return false;
      if (key.is_number && key.number >= 0)
        out[uint32_t(key.number)] = std::move(value);
    }
    return true;
  }
};

bool parse_payload(std::span<const uint8_t> payload, OpenPrintTag &out) {
  out.payload.assign(payload.begin(), payload.end());
  OptRegion meta;
  Cbor meta_cbor{payload};
  if (!meta_cbor.map(meta))
    return false;
  auto offset = [&](uint32_t key, size_t fallback) {
    auto it = meta.find(key);
    return it != meta.end() && it->second.is_number ? size_t(it->second.number) : fallback;
  };
  // Region sizes only matter for writes, CBOR maps delimit themselves
  const size_t main_offset = offset(0, meta_cbor.pos);
  if (main_offset >= payload.size())
    return false;
  Cbor main_cbor{payload.subspan(main_offset)};
  if (!main_cbor.map(out.main))
    return false;
  const size_t aux_offset = offset(2, payload.size());
  if (aux_offset < payload.size()) {
    // the aux region spans its size, or till the main region after it, or the payload end
    const size_t end = main_offset > aux_offset ? main_offset : payload.size();
    out.aux_offset = aux_offset;
    out.aux_size = std::min(offset(3, end - aux_offset), end - aux_offset);
    Cbor aux_cbor{payload.subspan(aux_offset)};
    if (!aux_cbor.map(out.aux))
      out.aux.clear();  // a blank aux region is not an error
  }
  return true;
}

bool parse_ndef(std::span<const uint8_t> msg, const uint8_t *mem, OpenPrintTag &out) {
  size_t pos = 0;
  while (pos + 3 <= msg.size()) {
    const uint8_t header = msg[pos++];
    const uint8_t type_length = msg[pos++];
    size_t payload_length;
    if (header & 0x10) {  // short record
      payload_length = msg[pos++];
    } else {
      if (pos + 4 > msg.size())
        return false;
      payload_length = (uint32_t(msg[pos]) << 24) | (uint32_t(msg[pos + 1]) << 16) | (uint32_t(msg[pos + 2]) << 8) |
                       msg[pos + 3];
      pos += 4;
    }
    size_t id_length = 0;
    if (header & 0x08) {
      if (pos >= msg.size())
        return false;
      id_length = msg[pos++];
    }
    if (pos + type_length + id_length + payload_length > msg.size())
      return false;
    const std::string_view type(reinterpret_cast<const char *>(&msg[pos]), type_length);
    pos += type_length + id_length;
    if ((header & 0x07) == TNF_MIME && type == MIME_TYPE) {
      out.payload_offset = &msg[pos] - mem;
      return parse_payload(msg.subspan(pos, payload_length), out);
    }
    pos += payload_length;
    if (header & 0x40)  // message end
      break;
  }
  return false;
}

}  // namespace

size_t t5t_memory_size(std::span<const uint8_t> cc) {
  if (cc.size() < 4 || (cc[0] != 0xE1 && cc[0] != 0xE2))
    return 0;
  if (cc[2] != 0)
    return cc[2] * 8;
  // 8 byte CC: MLEN in bytes 6-7
  return cc.size() < 8 ? 0 : ((cc[6] << 8) | cc[7]) * 8;
}

bool parse_openprinttag(std::span<const uint8_t> mem, OpenPrintTag &out) {
  if (t5t_memory_size(mem) == 0)
    return false;
  size_t pos = mem[2] != 0 ? 4 : 8;
  while (pos < mem.size()) {
    const uint8_t type = mem[pos++];
    if (type == TLV_NULL)
      continue;
    if (type == TLV_TERMINATOR || pos >= mem.size())
      return false;
    size_t length = mem[pos++];
    if (length == 0xFF) {
      if (pos + 2 > mem.size())
        return false;
      length = (mem[pos] << 8) | mem[pos + 1];
      pos += 2;
    }
    if (pos + length > mem.size())
      return false;
    if (type == TLV_NDEF)
      return parse_ndef(mem.subspan(pos, length), mem.data(), out);
    pos += length;
  }
  return false;
}

}  // namespace esphome::pn5180

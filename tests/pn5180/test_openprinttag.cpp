// Host test of the OpenPrintTag parser:
//   g++ -std=c++20 -I components tests/pn5180/test_openprinttag.cpp components/pn5180/openprinttag.cpp && ./a.out
// sample_tag.bin is the spec's sample tag after its data_to_update.yaml (utils/rec_update.py)
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <vector>

#include "pn5180/openprinttag.h"

using namespace esphome::pn5180;

int main() {
  std::ifstream f("tests/pn5180/sample_tag.bin", std::ios::binary);
  std::vector<uint8_t> mem((std::istreambuf_iterator<char>(f)), {});
  assert(mem.size() == 304);
  assert(t5t_memory_size(mem) == 304);

  OpenPrintTag tag;
  assert(parse_openprinttag(mem, tag));
  assert(tag.main.at(9).number == 0);  // PLA
  assert(tag.main.at(10).bytes == "PLA Galaxy Black");
  assert(tag.main.at(11).bytes == "NOT Prusament");
  assert(tag.main.at(14).number == 1739371290);
  assert(tag.main.at(16).number == 1000);
  assert(tag.main.at(17).number == 1012);
  assert(tag.main.at(19).bytes == std::string("\x3d\x3e\x3d"));
  assert(std::fabs(tag.main.at(27).number - 0.2) < 0.001);  // half float
  assert(tag.main.at(28).bytes.empty());                     // array skipped
  assert(tag.main.at(35).number == 220);
  assert(tag.main.at(42).number == 75);  // last field after the array
  assert(tag.aux.at(0).number == 100);
  assert(tag.payload.size() == 261 && tag.payload[0] == 0xA1);  // meta {2: 226}

  // Truncated or foreign data must not parse
  OpenPrintTag bad;
  assert(!parse_openprinttag(std::span(mem).first(100), bad));
  mem[0] = 0x00;
  assert(!parse_openprinttag(mem, bad));

  // Deeply nested arrays are rejected instead of recursing: meta {}, main {0: [[[...]]]}
  std::string mime = "application/vnd.openprinttag";
  std::vector<uint8_t> deep = {0xA0, 0xA1, 0x00};
  deep.insert(deep.end(), 200, 0x81);
  deep.push_back(0x00);
  std::vector<uint8_t> nested = {0xE1, 0x40, 0x80, 0x01, 0x03, 0xFF, 0x00, uint8_t(3 + mime.size() + deep.size()),
                                 0xD2, uint8_t(mime.size()), uint8_t(deep.size())};
  nested.insert(nested.end(), mime.begin(), mime.end());
  nested.insert(nested.end(), deep.begin(), deep.end());
  assert(!parse_openprinttag(nested, bad));
  deep.assign({0xA0, 0xA1, 0x00, 0x81, 0x81, 0x00});
  nested.resize(11 + mime.size());
  nested[7] = uint8_t(3 + mime.size() + deep.size());
  nested[10] = uint8_t(deep.size());
  nested.insert(nested.end(), deep.begin(), deep.end());
  assert(parse_openprinttag(nested, bad));  // a shallow one parses

  std::puts("ok");
}

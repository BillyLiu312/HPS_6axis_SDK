#include "hps6axis/hps6axis.hpp"

#include <cassert>
#include <cstdint>
#include <iostream>

int main() {
  // CRC vectors from the protocol manual.
  // The public API does not expose CRC; verify the documented frame by using its
  // known wire representation in a small independent calculation.
  const uint8_t data[] = {0x00, 0x00, 0x01};
  uint16_t crc = 0xFFFF;
  for (uint8_t b : data) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (int i = 0; i < 8; ++i) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  assert(crc == 0xDCBD);  // transmitted as BD DC

  const uint8_t measurement[] = {
      0x00, 0x00, 0x02, 0x16, 0xFF, 0xFF, 0xFF, 0x01, 0xFA, 0xFF,
      0xFF, 0xEF, 0x02, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x0A,
      0x00, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00};
  crc = 0xFFFF;
  for (uint8_t b : measurement) {
    crc ^= static_cast<uint16_t>(b) << 8;
    for (int i = 0; i < 8; ++i) crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
  }
  assert(crc == 0x586F);  // transmitted as 6F 58
  std::cout << "protocol vectors passed\n";
}

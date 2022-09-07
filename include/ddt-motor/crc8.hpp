#ifndef DDT_MOTOR_CRC8_HPP_
#define DDT_MOTOR_CRC8_HPP_

#include <stdint.h>

#include <vector>

namespace ddt {

template <class I>
uint8_t dallas_crc8(I begin, I end) {
  uint8_t crc = 0;
  for (auto it = begin; it != end; ++it) {
    uint8_t inbyte = *it;
    for (uint8_t j = 0; j < 8; ++j) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

}  // namespace ddt

#endif
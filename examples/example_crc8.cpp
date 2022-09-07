#include <limits.h>  // CHAR_BIT
#include <stdint.h>
#include <stdio.h>

#include <array>
#include <vector>
#define MSB_CRCS (0x31)  // x8 + x5 + x4 + 1

static uint8_t calcCRC8CCITT(uint8_t* buff, size_t size) {
  uint8_t crc8;

  for (crc8 = 0xFF; size != 0; size--) {
    crc8 ^= *buff++;

    for (int idx = 0; idx < CHAR_BIT; idx++) {
      if (crc8 & 0x80) {
        crc8 <<= 1;
        crc8 ^= MSB_CRCS;
      } else {
        crc8 <<= 1;
      }
    }
  }
  return crc8;
}

uint8_t CalcCrc(uint8_t* data, int size) {
  uint8_t crc = 0xFF;
  for (int i = 0; i < size; i++) {
    crc ^= data[i];
    for (uint8_t bit = 8; bit > 0; --bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x31u;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

uint8_t get_crc8(uint8_t* p, int counter) {
  static const std::array<uint8_t, 256> CRC8Table = {
      0,   94,  188, 226, 97,  63,  221, 131, 194, 156, 126, 32,  163, 253, 31,
      65,  157, 195, 33,  127, 252, 162, 64,  30,  95,  1,   227, 189, 62,  96,
      130, 220, 35,  125, 159, 193, 66,  28,  254, 160, 225, 191, 93,  3,   128,
      222, 60,  98,  190, 224, 2,   92,  223, 129, 99,  61,  124, 34,  192, 158,
      29,  67,  161, 255, 70,  24,  250, 164, 39,  121, 155, 197, 132, 218, 56,
      102, 229, 187, 89,  7,   219, 133, 103, 57,  186, 228, 6,   88,  25,  71,
      165, 251, 120, 38,  196, 154, 101, 59,  217, 135, 4,   90,  184, 230, 167,
      249, 27,  69,  198, 152, 122, 36,  248, 166, 68,  26,  153, 199, 37,  123,
      58,  100, 134, 216, 91,  5,   231, 185, 140, 210, 48,  110, 237, 179, 81,
      15,  78,  16,  242, 172, 47,  113, 147, 205, 17,  79,  173, 243, 112, 46,
      204, 146, 211, 141, 111, 49,  178, 236, 14,  80,  175, 241, 19,  77,  206,
      144, 114, 44,  109, 51,  209, 143, 12,  82,  176, 238, 50,  108, 142, 208,
      83,  13,  239, 177, 240, 174, 76,  18,  145, 207, 45,  115, 202, 148, 118,
      40,  171, 245, 23,  73,  8,   86,  180, 234, 105, 55,  213, 139, 87,  9,
      235, 181, 54,  104, 138, 212, 149, 203, 41,  119, 244, 170, 72,  22,  233,
      183, 85,  11,  136, 214, 52,  106, 43,  117, 151, 201, 74,  20,  246, 168,
      116, 42,  200, 150, 21,  75,  169, 247, 182, 232, 10,  84,  215, 137, 107,
      53};
  uint8_t crc8 = 0x00;
  // Model:CRC-8/MAXIM
  // polynomial x8 + x5 + x4 + 1

  for (; counter > 0; counter--) {
    crc8 = CRC8Table[crc8 ^ *p];
    p++;
  }
  return (crc8);
}
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
uint8_t dallas_crc8(const uint8_t* data, const unsigned int size) {
  uint8_t crc = 0;
  for (unsigned int i = 0; i < size; ++i) {
    uint8_t inbyte = data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C;
      inbyte >>= 1;
    }
  }
  return crc;
}

int main(void) {
  uint8_t data[] = {0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x33};
  std::vector<uint8_t> vec(
      {0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x12, 0x34, 0x33});

  // crc = calcCRC8CCITT(data, 2);
  auto crc1 = get_crc8(data, 9);
  auto crc2 = CalcCrc(data, 9);
  auto crc3 = calcCRC8CCITT(data, 9);
  auto crc4 = dallas_crc8(data, 9);
  auto crc5 = dallas_crc8(vec.begin(), vec.end());

  printf("0x%x\n", crc1);
  printf("0x%x\n", crc2);
  printf("0x%x\n", crc3);
  printf("0x%x\n", crc4);
  printf("0x%x\n", crc5);

  return 0;
}

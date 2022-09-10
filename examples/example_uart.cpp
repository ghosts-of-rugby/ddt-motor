#include "ddt-motor/uart.hpp"

int main(int argc, char const *argv[]) {
  ddt::Uart uart("/dev/ttyUSB0", ddt::Uart::BaudRate::B_115200);
  uart.Send({0x12,0x60});
  auto data = uart.Receive();
  return 0;
}

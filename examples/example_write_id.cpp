// #include <format>
#include <iostream>

#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

int main(int argc, char const *argv[]) {
  using namespace ddt;
  uint8_t id = 0x00;

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  // ddt::Motor::SetID(id, uart);
  ddt::Motor::CheckID(uart);

  return 0;
}

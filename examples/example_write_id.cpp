// #include <format>
#include <iostream>

#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

int main(int argc, char const *argv[]) {
  using namespace ddt;  // NOLINT
  int id;

  std::cout << "Input ID : ";
  std::cin >> id;

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  ddt::Motor::SetID(id, uart);
  ddt::Motor::CheckID(uart);

  return 0;
}

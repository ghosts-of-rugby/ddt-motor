// #include <format>
#include <iostream>
#include <thread>

#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

int main(int argc, char const *argv[]) {
  using namespace ddt;
  uint8_t id = 0x01;

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  ddt::Motor motor0(uart, 0x00);
  ddt::Motor motor1(uart, 0x01);

  for (int i = 0; i < 50; i++) {
    motor1.Drive(20.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    motor0.Drive(10.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  for (int i = 0; i < 100; i++) {
    motor0.Drive(0.0);
    // motor1.Drive(0.0);
  }

  return 0;
}

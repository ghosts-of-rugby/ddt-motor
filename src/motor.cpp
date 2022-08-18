#include "ddt-motor/motor.hpp"

#include <iostream>
#include <thread>

#include "ddt-motor/crc8.hpp"

namespace ddt {

std::vector<Motor*> Motor::motors;

Motor::Motor(std::shared_ptr<Uart> uart, uint8_t id) : uart(uart), id(id) {
  motors.emplace_back(this);
}

void Motor::Drive(double drive_value) {
  uint16_t val = static_cast<uint16_t>(drive_value);
  std::vector<uint8_t> send = {
      id, 0x64, uint8_t(val >> 8), uint8_t(val & 0x00ff), 0, 0, 0, 0, 0};
  send.emplace_back(dallas_crc8(send.begin(), send.end()));
  uart->Send(send);
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  auto data = uart->Receive();
  if (data.size() != 0) {
    std::cout << int(id) << "," << int(data[0]) << std::endl;
    return;
  }
  std::cout << int(id) << "aaa" << std::endl;
  return;
  // throw std::runtime_error("ponyo");
}

void Motor::SetID(uint8_t id, std::shared_ptr<Uart> uart) {
  for (int i = 0; i < 100; i++) {
    uart->Send({{0xAA, 0x55, 0x53, id, 0, 0, 0, 0, 0, 0}});
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void Motor::CheckID(std::shared_ptr<Uart> uart) {
  std::vector<uint8_t> data = {0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0};

  data.emplace_back(dallas_crc8(data.begin(), data.end()));

  for (int i = 0; i < 10; i++) {
    uart->Send(data);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto received = uart->Receive();
    if (received.size() > 0) {
      std::cout << received.size() << std::endl;
      std::cout << int(received.back()) << ","
                << int(dallas_crc8(received.begin(), received.end() - 1))
                << std::endl;
      std::cout << int(received[0]) << std::endl;
      return;
    }
  }
  throw std::runtime_error("ponyo");
}

}  // namespace ddt
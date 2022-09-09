#include "ddt-motor/motor.hpp"

#include <cmath>
#include <iostream>
#include <thread>

#include "ddt-motor/crc8.hpp"

namespace ddt {

template <typename T = std::int16_t>
constexpr T combine(std::uint8_t a, std::uint8_t b) {
  return a << 8 | b;
}

template <class I>
bool check_data(I begin, I end) {
  return (*end) == dallas_crc8(begin, end - 1);
}

std::vector<Motor*> Motor::motors;

Motor::Motor(std::shared_ptr<Uart> uart, uint8_t id,
             std::chrono::milliseconds sleep_time)
    : uart(uart), id(id), sleep_time(sleep_time) {
  motors.emplace_back(this);
}

void Motor::SetMode(DriveMode mode) {
  std::vector<uint8_t> send = {id, 0xA0, 0, 0, 0,
                               0,  0,    0, 0, static_cast<uint8_t>(mode)};

  for (int i = 0; i < 10; i++) {
    uart->Send(send);
    std::this_thread::sleep_for(sleep_time);
    auto state = Observe();
    if (state.has_value() && state.value().mode == mode) {
      return;
    }
  }
  throw std::runtime_error("SetMode failed");
}

std::optional<Motor::State> Motor::DriveVelocity(double velocity, double acc,
                                                 bool brake) {
  int16_t val = static_cast<int16_t>(velocity * 60.0 / 2 / M_PI);
  std::vector<uint8_t> send = {id,    //
                               0x64,  //
                               uint8_t(val >> 8),
                               uint8_t(val & 0x00ff),
                               0,
                               0,
                               static_cast<uint8_t>(acc),
                               uint8_t((brake) ? 0xFF : 0x00),
                               0};
  send.emplace_back(dallas_crc8(send.begin(), send.end()));
  return Drive(send);
}

std::optional<Motor::State> Motor::DriveCurrent(double current) {
  int16_t val = static_cast<int16_t>(current * 32767.0 / 8.0);
  std::vector<uint8_t> send = {id,    //
                               0x64,  //
                               uint8_t(val >> 8),
                               uint8_t(val & 0x00ff),
                               0,
                               0,
                               0,
                               0,
                               0};
  send.emplace_back(dallas_crc8(send.begin(), send.end()));
  return Drive(send);
}

std::optional<Motor::State> Motor::Observe() {
  std::vector<uint8_t> send = {id, 0x74, 0, 0, 0, 0, 0, 0, 0};
  send.emplace_back(dallas_crc8(send.begin(), send.end()));
  uart->Send(send);
  std::this_thread::sleep_for(sleep_time);
  auto data = uart->Receive();

  if (data.size() == 10 && data[0] == id) {
    Motor::State state{
        .id = data[0],
        .mode = static_cast<Motor::DriveMode>(data[1]),
        .current = static_cast<double>(combine<int16_t>(data[2], data[3])) /
                   32767.0 * 8.0,
        .velocity = static_cast<double>(combine<int16_t>(data[4], data[5])) *
                    2 * M_PI / 60.0,
        .angle = static_cast<double>(data[7]) / 256.0 * 2 * M_PI,
        .stator_temperature = 0.0,
        .over_heat = false,
        .stall = false,
        .phase_over_current = false,
        .bus_over_currnet = false,
        .sensor_fault = false,
    };
    return state;
  }
  return std::nullopt;
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
      std::cout << "CRC8 Check : " << int(received.back()) << " == "
                << int(dallas_crc8(received.begin(), received.end() - 1))
                << std::endl;
      std::cout << "Written ID : " << int(received[0]) << std::endl;
      return;
    }
  }
  throw std::runtime_error("check id failed");
}

std::optional<Motor::State> Motor::Drive(std::vector<uint8_t> send_data) {
  uart->Send(send_data);
  std::this_thread::sleep_for(sleep_time);
  auto data = uart->Receive();

  if (data.size() == 10 && data[0] == id) {
    Motor::State state{
        .id = data[0],
        .mode = static_cast<Motor::DriveMode>(data[1]),
        .current = static_cast<double>(combine<int16_t>(data[2], data[3])) /
                   32767.0 * 8.0,
        .velocity = static_cast<double>(combine<int16_t>(data[4], data[5])) *
                    2 * M_PI / 60.0,
        .angle = static_cast<double>(combine<int16_t>(data[6], data[7])) /
                 32767.0 * 2 * M_PI,
        .stator_temperature = 0.0,
        .over_heat = false,
        .stall = false,
        .phase_over_current = false,
        .bus_over_currnet = false,
        .sensor_fault = false,
    };
    return state;
  }
  return std::nullopt;
}

}  // namespace ddt
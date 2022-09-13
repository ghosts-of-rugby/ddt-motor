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

Motor::Motor(std::shared_ptr<Uart> uart, uint8_t id,
             std::chrono::milliseconds sleep_time)
    : uart(uart), id(id), sleep_time(sleep_time) {}

void Motor::SetMode(DriveMode mode) {
  for (int i = 0; i < 10; i++) {
    SendSetModeCommand(mode);
    std::this_thread::sleep_for(sleep_time);
    auto state = Observe();
    if (state.has_value() && state.value().mode == mode) {
      return;
    }
  }
  throw std::runtime_error("SetMode failed");
}

void Motor::SendSetModeCommand(Motor::DriveMode mode) {
  std::vector<uint8_t> send = {id, 0xA0, 0, 0, 0,
                               0,  0,    0, 0, static_cast<uint8_t>(mode)};
  uart->Send(send);
}

void Motor::SendVelocityCommand(double velocity, double acc, bool brake) {
  int16_t val = static_cast<int16_t>(velocity * 60.0 / 2 / M_PI);
  std::vector<uint8_t> command = {id,    //
                                  0x64,  //
                                  uint8_t(val >> 8),
                                  uint8_t(val & 0x00ff),
                                  0,
                                  0,
                                  static_cast<uint8_t>(acc),
                                  uint8_t((brake) ? 0xFF : 0x00),
                                  0};
  command.emplace_back(dallas_crc8(command.begin(), command.end()));
  uart->Send(command);
}

void Motor::SendCurrentCommand(double current) {
  int16_t val = static_cast<int16_t>(current * 32767.0 / 8.0);
  std::vector<uint8_t> command = {id,    //
                                  0x64,  //
                                  uint8_t(val >> 8),
                                  uint8_t(val & 0x00ff),
                                  0,
                                  0,
                                  0,
                                  0,
                                  0};
  command.emplace_back(dallas_crc8(command.begin(), command.end()));
  uart->Send(command);
}

void Motor::SendObserveCommand() {
  std::vector<uint8_t> command = {id, 0x74, 0, 0, 0, 0, 0, 0, 0};
  command.emplace_back(dallas_crc8(command.begin(), command.end()));
  uart->Send(command);
}

std::optional<Motor::State> Motor::ReceiveSpinMotorFeedback() {
  auto data = uart->Receive();
  if (data.size() == 10 && data[0] == id) {
    uint8_t fault_value = data[8];
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
        // .over_heat = false,
        // .stall = false,
        // .phase_over_current = false,
        // .bus_over_currnet = false,
        // .sensor_fault = false,
        .over_heat = static_cast<bool>(fault_value & 0b00010000),
        .stall = static_cast<bool>(fault_value & 0b00001000),
        .phase_over_current = static_cast<bool>(fault_value & 0b00000100),
        .bus_over_currnet = static_cast<bool>(fault_value & 0b00000010),
        .sensor_fault = static_cast<bool>(fault_value & 0b00000001),
    };
    return state;
  }
  return std::nullopt;
}

std::optional<Motor::State> Motor::ReceiveObserveFeedback() {
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

std::optional<Motor::State> Motor::DriveVelocity(double velocity, double acc,
                                                 bool brake) {
  SendVelocityCommand(velocity, acc, brake);
  std::this_thread::sleep_for(sleep_time);
  return ReceiveSpinMotorFeedback();
}

std::optional<Motor::State> Motor::DriveCurrent(double current) {
  SendCurrentCommand(current);
  std::this_thread::sleep_for(sleep_time);
  return ReceiveSpinMotorFeedback();
}

std::optional<Motor::State> Motor::Observe() {
  SendObserveCommand();
  std::this_thread::sleep_for(sleep_time);
  return ReceiveObserveFeedback();
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

}  // namespace ddt
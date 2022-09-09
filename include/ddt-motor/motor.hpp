#ifndef DDT_MOTOR_MOTOR_HPP_
#define DDT_MOTOR_MOTOR_HPP_

#include <stdint.h>

#include <chrono>
#include <memory>
#include <optional>
#include <vector>

#include "ddt-motor/uart.hpp"

namespace ddt {

using namespace std::chrono_literals;  // NOLINT

class Motor {
 private:
  static std::vector<Motor*> motors;
  std::shared_ptr<Uart> uart;
  uint8_t id;
  std::chrono::milliseconds sleep_time;

 public:
  enum class DriveMode : uint8_t {
    Current = 0x01,
    Velocity = 0x02,
    Angle = 0x03,
  };
  struct State {
    uint8_t id;
    DriveMode mode;
    double current;
    double velocity;
    double angle;
    std::optional<double> stator_temperature;

    // Fault Value
    bool over_heat;
    bool stall;
    bool phase_over_current;
    bool bus_over_currnet;
    bool sensor_fault;
  };

  static void SetID(uint8_t id, std::shared_ptr<Uart> uart);

  static void CheckID(std::shared_ptr<Uart> uart);

  Motor(std::shared_ptr<Uart> uart, uint8_t id,
        std::chrono::milliseconds sleep_time = 5ms);

  std::optional<State> DriveVelocity(double velocity, double acc = 0.0,
                                     bool brake = false);

  std::optional<State> DriveCurrent(double current);

  void SetMode(DriveMode mode);

  std::optional<State> Observe();

 private:
  std::optional<State> Drive(std::vector<uint8_t> data);
};

}  // namespace ddt

#endif  // DDT_MOTOR_MOTOR_HPP_
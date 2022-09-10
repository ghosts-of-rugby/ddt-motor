#ifndef DDT_MOTOR_MOTOR_HPP_
#define DDT_MOTOR_MOTOR_HPP_

#include <stdint.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "ddt-motor/uart.hpp"

namespace ddt {

using namespace std::chrono_literals;  // NOLINT

class Motor {
 private:
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

 public:
  static void SetID(uint8_t id, std::shared_ptr<Uart> uart);

  static void CheckID(std::shared_ptr<Uart> uart);

  Motor(std::shared_ptr<Uart> uart, uint8_t id,
        std::chrono::milliseconds sleep_time = 5ms);

  void SendSetModeCommand(DriveMode mode);
  void SendVelocityCommand(double velocity, double acc = 0.0,
                           bool brake = false);
  void SendCurrentCommand(double current);
  void SendObserveCommand();

  std::optional<State> ReceiveSpinMotorFeedback();
  std::optional<State> ReceiveObserveFeedback();

  Uart::Packet GetObserveCommand();
  Uart::Packet GetSetModeCommand();

  std::optional<State> DriveVelocity(double velocity, double acc = 0.0,
                                     bool brake = false);

  std::optional<State> DriveCurrent(double current);

  void SetMode(DriveMode mode);

  std::optional<State> Observe();
};

}  // namespace ddt

#endif  // DDT_MOTOR_MOTOR_HPP_
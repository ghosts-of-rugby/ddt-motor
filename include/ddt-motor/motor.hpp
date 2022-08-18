#ifndef DDT_MOTOR_MOTOR_HPP_
#define DDT_MOTOR_MOTOR_HPP_

#include <stdint.h>

#include <memory>
#include <optional>

#include "ddt-motor/uart.hpp"

namespace ddt {

class Motor {
 private:
  static std::vector<Motor*> motors;
  std::shared_ptr<Uart> uart;
  uint8_t id;

 public:
  enum class DriveMode { Current, Velocity, Angle };
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

  const static Uart::BaudRate uart_baudrate = Uart::BaudRate::B_115200;

  static void SetID(uint8_t id, std::shared_ptr<Uart> uart);

  static void CheckID(std::shared_ptr<Uart> uart);

  Motor(std::shared_ptr<Uart> uart, uint8_t id);

  void Drive(double drive_value);

  void Brake();

  static void BreakAll();
};

}  // namespace ddt

#endif
// #include <format>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

double clip(double x, double min, double max) {
  if (x < min) {
    return min;
  } else if (x > max) {
    return max;
  }
  return x;
}

int main(int argc, char const *argv[]) {
  using namespace ddt;  // NOLINT
  using second = std::chrono::duration<double>;

  std::ofstream ofs("logs/drive_double_motor_log.csv");

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  ddt::Motor motor0(uart, 0x03);
  ddt::Motor motor1(uart, 0x06);
  ddt::Motor motor2(uart, 0x09);

  ddt::AngleFilter filter0;
  ddt::AngleFilter filter1;
  ddt::AngleFilter filter2;

  motor1.SetMode(ddt::Motor::DriveMode::Velocity);
  motor2.SetMode(ddt::Motor::DriveMode::Velocity);

  double motor1_input = 0.0;
  double motor2_input = 0.0;

  double motor1_angle_ref = -7.0;
  double motor2_angle_ref = 7.0;

  double motor1_velocity_max = 5.0;
  double motor2_velocity_max = 5.0;

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < 1000; i++) {
    auto state1 = motor1.DriveVelocity(motor1_input);
    auto state2 = motor2.DriveVelocity(motor2_input);

    if (state1.has_value() && state2.has_value()) {
      auto elapsed = std::chrono::duration_cast<second>(
          std::chrono::high_resolution_clock::now() - start);
      double angle1 = filter1.Update(state1->angle);
      double angle2 = filter2.Update(state2->angle);

      motor1_input = -5 * (motor1_angle_ref - angle1);
      motor2_input = -5 * (motor2_angle_ref - angle2);

      motor1_input =
          clip(motor1_input, -motor1_velocity_max, motor1_velocity_max);
      motor2_input =
          clip(motor2_input, -motor2_velocity_max, motor2_velocity_max);

      ofs << elapsed.count() << "," << angle1 << "," << state1->velocity << ","
          << angle2 << "," << state2->velocity << std::endl;
    } else {
      std::cout << "connection error!!" << std::endl;
    }
  }
  auto state1 = motor1.DriveVelocity(0.0);
  auto state2 = motor2.DriveVelocity(0.0);
  return 0;
}

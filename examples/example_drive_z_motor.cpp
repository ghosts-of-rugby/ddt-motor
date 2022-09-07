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

  std::ofstream ofs("logs/drive_log.csv");

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  ddt::Motor motor0(uart, 0x03, 3ms);

  ddt::AngleFilter filter0;

  motor0.SetMode(ddt::Motor::DriveMode::Velocity);
  double input = 0.0;

  double ref_angle = 10.0;
  double max_velocity = 10.0;

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < 1000; i++) {
    auto state = motor0.DriveVelocity(input, 0.0);
    if (state.has_value()) {
      auto elapsed = std::chrono::duration_cast<second>(
          std::chrono::high_resolution_clock::now() - start);
      double angle = filter0.Update(state->angle);

      input = -5.0 * (ref_angle - angle);
      input = clip(input, -max_velocity, max_velocity);

      ofs << elapsed.count() << "," << angle << "," << state->velocity << ","
          << input << std::endl;
    } else {
      std::cout << "conncection error !!" << std::endl;
    }
  }
  motor0.DriveVelocity(0.0, 0.0);
  return 0;
}

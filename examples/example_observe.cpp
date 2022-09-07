#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

int main(int argc, char const* argv[]) {
  using namespace ddt;  // NOLINT
  using second = std::chrono::duration<double>;

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  std::vector<ddt::Motor> motors = {
      // ddt::Motor(uart, 0x03),
      ddt::Motor(uart, 0x06, 4ms), ddt::Motor(uart, 0x09, 4ms)};

  std::vector<ddt::AngleFilter> filters = {
      ddt::AngleFilter(), ddt::AngleFilter(), ddt::AngleFilter()};

  std::ofstream ofs("logs/angle_log.csv");

  for (int j = 0; j < 2; j++) {
    motors[j].SetMode(ddt::Motor::DriveMode::Velocity);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; true; i++) {
    auto elapsed = std::chrono::duration_cast<second>(
        std::chrono::high_resolution_clock::now() - start);
    ofs << elapsed.count() << ",";
    for (int j = 0; j < motors.size(); j++) {
      auto state = motors[j].Observe();
      if (state.has_value()) {
        double angle = filters[j].Update(state->angle);
        ofs << angle << "," << state->velocity;
      } else {
        ofs << "NaN,NaN";
      }
      if (j != motors.size() - 1) {
        ofs << ",";
      }
    }
    ofs << std::endl;
  }

  return 0;
}

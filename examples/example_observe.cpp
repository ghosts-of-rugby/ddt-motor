#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

int main(int argc, char const* argv[]) {
  using namespace ddt;  // NOLINT

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  std::vector<ddt::Motor> motors = {
      // ddt::Motor(uart, 0x03),
      ddt::Motor(uart, 0x06, 4ms), ddt::Motor(uart, 0x09, 4ms)};

  std::vector<ddt::AngleFilter> filters = {
      ddt::AngleFilter(), ddt::AngleFilter(), ddt::AngleFilter()};

  // std::ofstream ofs("logs/angle_log.csv");

  for (int j = 0; j < 2; j++) {
    motors[j].SetMode(ddt::Motor::DriveMode::Velocity);
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));
  for (int i = 0; true; i++) {
    for (int j = 0; j < 2; j++) {
      auto state = motors[j].Observe();
      // if (state.has_value()) {
      //   double angle = filters[j].Update(state->angle);
      //   std::cout << state->velocity << ",";
      // } else {
      //   std::cout << "NaN,";
      // }
    }
    // std::cout << std::endl;
  }

  return 0;
}

#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/observer.hpp"
#include "ddt-motor/uart.hpp"

int main(int argc, char const* argv[]) {
  using namespace ddt;  // NOLINT
  using second = std::chrono::duration<double>;

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  ddt::Motor motor_theta(uart, 0x06, 5ms);
  ddt::Observer observer(0.005, 7.2, -10, -20);
  ddt::AngleFilter angle_filter_theta;

  double current = 0.0;
  current = 2.0;
  double max_current = 1.0;

  // reference
  double ref_angle = -3.0;
  double ref_velocity = 0.0;

  std::ofstream ofs("logs/theta_motor_log.csv");

  motor_theta.SetMode(ddt::Motor::DriveMode::Current);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; true; i++) {
    // auto state = motor_theta.DriveVelocity(1.0);
    // auto state = motor_theta.DriveCurrent(current);
    auto state = motor_theta.DriveCurrent(current);

    if (state.has_value()) {
      auto elapsed = std::chrono::duration_cast<second>(
          std::chrono::high_resolution_clock::now() - start);
      double angle = angle_filter_theta.Update(state->angle);

      auto [vel_est, dis_est] = observer.Update(current, state->velocity);

      current =
          0.5 * (ref_velocity - vel_est) - 0.5 * (ref_angle - angle) + dis_est;
      current = std::clamp(current, -max_current, max_current);

      ofs << elapsed.count() << "," << angle << "," << state->velocity << ","
          << state->current << "," << vel_est << "," << dis_est << std::endl;
    } else {
      std::cout << "error" << std::endl;
    }
  }

  return 0;
}

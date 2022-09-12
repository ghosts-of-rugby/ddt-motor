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

  auto delay = 10ms;

  auto uart = std::make_shared<ddt::Uart>("/dev/ttyUSB0",
                                          ddt::Uart::BaudRate::B_115200);

  ddt::Motor motor_theta(uart, 0x06, delay);
  ddt::Observer observer(15.7, -20, -10);
  ddt::AngleFilter angle_filter_theta;

  double current = 0.0;
  double max_current = 3.0;

  // reference
  // double ref_angle = -6.0;
  // double ref_velocity = 0.0;
  double ref_velocity = 6.0;
  bool flag = false;

  std::ofstream ofs("logs/theta_motor_log.csv");

  motor_theta.SetMode(ddt::Motor::DriveMode::Current);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  std::cout << "start" << std::endl;
  auto start = std::chrono::high_resolution_clock::now();
  auto pre_update_time = start;

  for (int i = 0; true; i++) {
    motor_theta.SendCurrentCommand(current);
    // motor_theta.SendObserveCommand();
    std::this_thread::sleep_for(delay);
    auto state = motor_theta.ReceiveSpinMotorFeedback();
    // auto state = motor_theta.ReceiveObserveFeedback();

    if (state.has_value()) {
      auto now = std::chrono::high_resolution_clock::now();
      auto dt = std::chrono::duration_cast<second>(now - pre_update_time);
      pre_update_time = now;
      auto elapsed = std::chrono::duration_cast<second>(now - start);

      double ref_angle = -ref_velocity * elapsed.count();

      if (ref_angle < -10) {
        flag = true;
      }
      if (flag) {
        ref_angle = -10.0;
        ref_velocity = 0.0;
      }

      double angle = angle_filter_theta.Update(state->angle);

      auto [vel_est, dis_est] = observer.Update(current, state->velocity, dt);

      current = 0.3 * (ref_velocity - state->velocity) -
                0.3 * (ref_angle - angle) + dis_est;
      current = std::clamp(current, -max_current, max_current);

      ofs << elapsed.count() << "," << angle << "," << ref_angle << ","
          << state->velocity << "," << vel_est << "," << ref_velocity << ","
          << state->current << "," << current << "," << dis_est << std::endl;
      // ofs << elapsed.count() << "," << angle << "," << state->velocity << ","
      //     << state->current << std::endl;
    } else {
      std::cout << "error" << std::endl;
    }
  }

  return 0;
}

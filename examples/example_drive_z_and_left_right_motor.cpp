// #include <format>
#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>

#include "ddt-motor/angle_filter.hpp"
#include "ddt-motor/motor.hpp"
#include "ddt-motor/uart.hpp"

class MotorController {};

int main(int argc, char const *argv[]) {
  using namespace ddt;  // NOLINT
  using second = std::chrono::duration<double>;

  std::ofstream ofs("logs/drive_z_and_left_right_motor_log.csv");

  auto uart_right_left = std::make_shared<ddt::Uart>(
      "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ0287SJ-if00-port0",
      ddt::Uart::BaudRate::B_115200);
  ddt::Motor motor_right(uart_right_left, 0x06);
  ddt::Motor motor_left(uart_right_left, 0x09);

  auto uart_z = std::make_shared<ddt::Uart>(
      "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AQ027PCT-if00-port0",
      ddt::Uart::BaudRate::B_115200);

  ddt::Motor motor_z(uart_z, 0x03);

  ddt::AngleFilter filter_right;
  ddt::AngleFilter filter_left;
  ddt::AngleFilter filter_z;

  motor_right.SetMode(ddt::Motor::DriveMode::Velocity);
  motor_left.SetMode(ddt::Motor::DriveMode::Velocity);
  motor_z.SetMode(ddt::Motor::DriveMode::Velocity);

  double motor_right_input = 0.0;
  double motor_left_input = 0.0;
  double motor_z_input = 0.0;

  double motor_right_angle_ref = -7.0;
  double motor_left_angle_ref = 7.0;
  double motor_z_angle_ref = 10.0;

  double max_velocity_right_left = 5.0;
  double max_velocity_z = 10.0;

  auto start = std::chrono::high_resolution_clock::now();

  for (int i = 0; i < 1000; i++) {
    auto delay = 4ms;
    motor_right.SendVelocityCommand(motor_right_input);
    motor_z.SendVelocityCommand(motor_z_input);
    std::this_thread::sleep_for(delay);
    auto state1 = motor_right.ReceiveSpinMotorFeedback();
    auto state = motor_z.ReceiveSpinMotorFeedback();

    motor_left.SendVelocityCommand(motor_left_input);
    std::this_thread::sleep_for(delay);
    auto state2 = motor_left.ReceiveSpinMotorFeedback();

    if (state1.has_value() && state2.has_value() && state.has_value()) {
      auto elapsed = std::chrono::duration_cast<second>(
          std::chrono::high_resolution_clock::now() - start);
      double angle1 = filter_right.Update(state1->angle);
      double angle2 = filter_left.Update(state2->angle);
      double angle = filter_z.Update(state->angle);

      motor_right_input = -5.0 * (motor_right_angle_ref - angle1);
      motor_left_input = -5.0 * (motor_left_angle_ref - angle2);
      motor_z_input = -5.0 * (motor_z_angle_ref - angle);

      motor_right_input = std::clamp(
          motor_right_input, -max_velocity_right_left, max_velocity_right_left);
      motor_left_input = std::clamp(motor_left_input, -max_velocity_right_left,
                                    max_velocity_right_left);
      motor_z_input =
          std::clamp(motor_z_input, -max_velocity_z, max_velocity_z);

      ofs << elapsed.count() << "," << angle1 << "," << state1->velocity << ","
          << angle2 << "," << state2->velocity << std::endl;
    } else {
      std::cout << "connection error!!" << std::endl;
    }
  }
  auto state1 = motor_right.DriveVelocity(0.0, 0.0, true);
  auto state2 = motor_left.DriveVelocity(0.0, 0.0, true);
  return 0;
}

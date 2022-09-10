#ifndef DDT_MOTOR_UART_HPP_
#define DDT_MOTOR_UART_HPP_

// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
#include <termios.h>
#include <unistd.h>

#include <string>
#include <vector>

namespace ddt {

class Uart {
 public:
  using Packet = std::vector<uint8_t>;
  enum class BaudRate : speed_t {
    B_9600 = B9600,
    B_19200 = B19200,
    B_115200 = B115200,
    B_460800 = B460800,
    B_921600 = B921600,
    B_1000000 = B1000000,
    B_1152000 = B1152000,
    B_1500000 = B1500000,
    B_2000000 = B2000000,
    B_2500000 = B2500000,
    B_3000000 = B3000000,
    B_3500000 = B3500000,
    B_4000000 = B4000000,
  };

 private:
  const char* dev;          // device
  const BaudRate baudrate;  // baudrate
  int fd;                   // file discriptor

 public:
  Uart(std::string dev = "/dev/ttyUSB0",
       BaudRate baudrate = BaudRate::B_115200);
  ~Uart();

  void Send(Packet data);
  Packet Receive();

  void Open();
};

}  // namespace ddt

#endif  // DDT_MOTOR_UART_HPP_
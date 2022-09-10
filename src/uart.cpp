#include "ddt-motor/uart.hpp"

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>

#include <stdexcept>
#include <thread>

#include "ddt-motor/myformat.hpp"

// https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/

namespace ddt {

Uart::Uart(std::string dev, BaudRate baudrate)
    : dev(dev.c_str()), baudrate(baudrate), fd(-1) {
  Open();
}
Uart::~Uart() { close(fd); }

void Uart::Open() {
  termios tty;

  for (int i = 0;; i++) {
    fd = open(dev, O_RDWR);
    if (fd == -1) {
      if (i == 0) std::cout << "Connect USB to {}"_fmt(dev) << std::endl;
    } else {
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  cfsetispeed(&tty, static_cast<speed_t>(baudrate));
  cfsetospeed(&tty, static_cast<speed_t>(baudrate));
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                           // communication (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all bits that set the data size
  tty.c_cflag |= CS8;      // 8 bits per byte (most common)
  tty.c_cflag &=
      ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      CREAD | CLOCAL;  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);  // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                          // (e.g. newline chars)
  tty.c_oflag &=
      ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  // blocking read of any number of chars with a maximum timeout
  tty.c_cc[VTIME] = 1;
  tty.c_cc[VMIN] = 0;

  cfmakeraw(&tty);

  // tcflush(fd, TCIFLUSH);
  if (tcsetattr(fd, TCSANOW, &tty) != 0) {
    throw std::runtime_error(
        "Error from {},{} : tcsetattr({}, TCSANOW, &tty) failed"_fmt(
            __FILE__, __LINE__, fd));
  }
  struct serial_struct serial_setting;
  ioctl(fd, TIOCGSERIAL, &serial_setting);
  serial_setting.flags |= ASYNC_LOW_LATENCY;
  ioctl(fd, TIOCSSERIAL, &serial_setting);
  return;
}

void Uart::Send(Uart::Packet data) {
  size_t size = write(fd, data.data(), data.size());
  if (size != data.size()) {
    throw std::runtime_error(
        "Error from {},{} : tcsetattr({}, TCSANOW, &tty) failed"_fmt(
            __FILE__, __LINE__, fd));
  }
}

Uart::Packet Uart::Receive() {
  // read size of in buffer
  std::size_t available_size = 0;
  ioctl(fd, FIONREAD, &available_size);
  std::vector<uint8_t> data(available_size);
  size_t size = read(fd, data.data(), available_size);
  return data;
}

}  // namespace ddt

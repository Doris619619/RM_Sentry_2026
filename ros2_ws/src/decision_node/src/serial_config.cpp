#include "decision_node/serial_config.hpp"

#include <stdexcept>
#include <string>

namespace decision_node::serial
{

speed_t baudToTermios(const int baudrate)
{
  switch (baudrate) {
    case 115200: return B115200;
#ifdef B921600
    case 921600: return B921600;
#else
    case 921600:
      throw std::invalid_argument("921600 baud is unsupported by this Linux termios implementation");
#endif
    default: throw std::invalid_argument("unsupported serial baudrate: " + std::to_string(baudrate));
  }
}

void configureRaw8N1(termios& options, const int baudrate)
{
  cfmakeraw(&options);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
#ifdef CRTSCTS
  options.c_cflag &= ~CRTSCTS;
#endif
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  const auto speed = baudToTermios(baudrate);
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
}

}  // namespace decision_node::serial

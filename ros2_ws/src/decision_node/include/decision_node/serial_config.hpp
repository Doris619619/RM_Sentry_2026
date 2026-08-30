#pragma once

#include <termios.h>

namespace decision_node::serial
{

speed_t baudToTermios(int baudrate);
void configureRaw8N1(termios& options, int baudrate);

}  // namespace decision_node::serial

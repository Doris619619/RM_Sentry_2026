#include "decision_node/serial_config.hpp"

#include <gtest/gtest.h>

#include <stdexcept>

TEST(SerialConfig, MapsSupportedBaudrates)
{
  EXPECT_EQ(decision_node::serial::baudToTermios(115200), B115200);
#ifdef B921600
  EXPECT_EQ(decision_node::serial::baudToTermios(921600), B921600);
#else
  EXPECT_THROW(decision_node::serial::baudToTermios(921600), std::invalid_argument);
#endif
}

TEST(SerialConfig, AppliesProductionRaw8N1WithoutFlowControl)
{
  termios options{};
  options.c_cflag = PARENB | CSTOPB | CS7;
#ifdef CRTSCTS
  options.c_cflag |= CRTSCTS;
#endif
  options.c_iflag = IXON | IXOFF | IXANY;

  decision_node::serial::configureRaw8N1(options, 921600);

  EXPECT_EQ(options.c_cflag & CSIZE, CS8);
  EXPECT_EQ(options.c_cflag & PARENB, 0U);
  EXPECT_EQ(options.c_cflag & CSTOPB, 0U);
  EXPECT_NE(options.c_cflag & CLOCAL, 0U);
  EXPECT_NE(options.c_cflag & CREAD, 0U);
#ifdef CRTSCTS
  EXPECT_EQ(options.c_cflag & CRTSCTS, 0U);
#endif
  EXPECT_EQ(options.c_iflag & (IXON | IXOFF | IXANY), 0U);
#ifdef B921600
  EXPECT_EQ(cfgetispeed(&options), B921600);
  EXPECT_EQ(cfgetospeed(&options), B921600);
#endif
}

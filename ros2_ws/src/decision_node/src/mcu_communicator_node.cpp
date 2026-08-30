#include "decision_node/mcu_protocol.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <memory>
#include <stdexcept>
#include <termios.h>
#include <unistd.h>

namespace
{
using namespace std::chrono_literals;
using QoS = rclcpp::QoS;

speed_t baudToTermios(const int baud)
{
  switch (baud) {
    case 115200: return B115200;
#ifdef B921600
    case 921600: return B921600;
#endif
    default: throw std::invalid_argument("unsupported serial baudrate: " + std::to_string(baud));
  }
}
}

class McuCommunicator final : public rclcpp::Node
{
public:
  McuCommunicator() : Node("mcu_communicator")
  {
    serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    baudrate_ = declare_parameter<int>("baudrate", 921600);
    nav_frequency_ = declare_parameter<double>("nav_frequency", 50.0);
    cmd_vel_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    reconnect_interval_ = declare_parameter<double>("reconnect_interval", 2.0);
    const auto reliable = QoS(10).reliable();
    const auto state = QoS(1).reliable();

    game_progress_ = create_publisher<std_msgs::msg::UInt8>("/referee/game_progress", state);
    remain_hp_ = create_publisher<std_msgs::msg::UInt16>("/referee/remain_hp", state);
    bullet_remain_ = create_publisher<std_msgs::msg::UInt16>("/referee/bullet_remain", state);
    occupy_status_ = create_publisher<std_msgs::msg::UInt8>("/referee/occupy_status", state);
    robot_id_ = create_publisher<std_msgs::msg::UInt8>("/robot/robot_id", state);
    robot_color_ = create_publisher<std_msgs::msg::UInt8>("/robot/robot_color", state);
    self_hp_ = create_publisher<std_msgs::msg::UInt16>("/robot/self_hp", state);
    self_max_hp_ = create_publisher<std_msgs::msg::UInt16>("/robot/self_max_hp", state);
    red_1_hp_ = create_publisher<std_msgs::msg::UInt16>("/referee/red_1_hp", state);
    red_3_hp_ = create_publisher<std_msgs::msg::UInt16>("/referee/red_3_hp", state);
    red_7_hp_ = create_publisher<std_msgs::msg::UInt16>("/referee/red_7_hp", state);
    blue_1_hp_ = create_publisher<std_msgs::msg::UInt16>("/referee/blue_1_hp", state);
    blue_3_hp_ = create_publisher<std_msgs::msg::UInt16>("/referee/blue_3_hp", state);
    blue_7_hp_ = create_publisher<std_msgs::msg::UInt16>("/referee/blue_7_hp", state);
    red_dead_ = create_publisher<std_msgs::msg::UInt16>("/referee/red_dead", state);
    blue_dead_ = create_publisher<std_msgs::msg::UInt16>("/referee/blue_dead", state);
    enemy_hero_ = create_publisher<geometry_msgs::msg::Point>("/enemy/hero_position", state);
    enemy_engineer_ = create_publisher<geometry_msgs::msg::Point>("/enemy/engineer_position", state);
    enemy_std3_ = create_publisher<geometry_msgs::msg::Point>("/enemy/standard_3_position", state);
    enemy_std4_ = create_publisher<geometry_msgs::msg::Point>("/enemy/standard_4_position", state);
    enemy_sentry_ = create_publisher<geometry_msgs::msg::Point>("/enemy/sentry_position", state);
    suggested_target_ = create_publisher<std_msgs::msg::UInt8>("/radar/suggested_target", state);
    radar_flags_ = create_publisher<std_msgs::msg::UInt16>("/radar/radar_flags", state);
    can_free_revive_ = create_publisher<std_msgs::msg::UInt8>("/sentry/can_free_revive", state);
    can_instant_revive_ = create_publisher<std_msgs::msg::UInt8>("/sentry/can_instant_revive", state);
    operator_input_ = create_publisher<geometry_msgs::msg::Point>("/operator/input", state);
    hurt_info_ = create_publisher<std_msgs::msg::UInt8>("/referee/hurt_info", state);

    cmd_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", reliable, [this](geometry_msgs::msg::Twist::ConstSharedPtr message) {
        vx_ = static_cast<float>(message->linear.x); vy_ = static_cast<float>(message->linear.y);
        wz_ = static_cast<float>(message->angular.z); last_cmd_vel_ = std::chrono::steady_clock::now();
      });
    motion_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "/motion", reliable, [this](std_msgs::msg::UInt8::ConstSharedPtr message) { motion_mode_ = message->data; sendMotion(); });
    recover_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "/recover", reliable, [this](std_msgs::msg::UInt8::ConstSharedPtr message) { heroes_never_die_ = message->data; });
    bullet_up_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "/bullet_up", reliable, [this](std_msgs::msg::UInt8::ConstSharedPtr message) { bullet_up_ = message->data ? 1 : 0; sendMotion(); });
    bullet_num_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "/bullet_num", reliable, [this](std_msgs::msg::UInt8::ConstSharedPtr message) { bullet_num_ = message->data; sendMotion(); });

    receive_timer_ = create_wall_timer(5ms, [this] { pollReceive(); });
    const auto nav_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / nav_frequency_));
    navigation_timer_ = create_wall_timer(nav_period, [this] { sendNavigation(); });
  }

  ~McuCommunicator() override { closeSerial(); }

private:
  bool ensureOpen()
  {
    if (fd_ >= 0) return true;
    const auto current = std::chrono::steady_clock::now();
    if (last_open_attempt_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(current - last_open_attempt_).count() < reconnect_interval_) return false;
    last_open_attempt_ = current;
    try {
      const int fd = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
      if (fd < 0) throw std::runtime_error(std::strerror(errno));
      termios options{};
      if (tcgetattr(fd, &options) != 0) { ::close(fd); throw std::runtime_error(std::strerror(errno)); }
      cfmakeraw(&options);
      const auto speed = baudToTermios(baudrate_);
      cfsetispeed(&options, speed); cfsetospeed(&options, speed);
      options.c_cflag |= (CLOCAL | CREAD);
      if (tcsetattr(fd, TCSANOW, &options) != 0) { ::close(fd); throw std::runtime_error(std::strerror(errno)); }
      fd_ = fd;
      RCLCPP_INFO(get_logger(), "opened %s at %d baud", serial_port_.c_str(), baudrate_);
      return true;
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "serial open failed for %s: %s", serial_port_.c_str(), error.what());
      return false;
    }
  }

  void closeSerial()
  {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; parser_.reset(); }
  }

  void writeFrame(const std::vector<uint8_t>& frame)
  {
    if (!ensureOpen()) return;
    const auto written = ::write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
      RCLCPP_WARN(get_logger(), "serial write failed: %s", written < 0 ? std::strerror(errno) : "short write");
      closeSerial();
    }
  }

  void pollReceive()
  {
    if (!ensureOpen()) return;
    uint8_t bytes[256];
    const auto count = ::read(fd_, bytes, sizeof(bytes));
    if (count > 0) for (const auto& frame : parser_.push(bytes, static_cast<std::size_t>(count))) publish(frame);
    if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_WARN(get_logger(), "serial read failed: %s", std::strerror(errno)); closeSerial();
    }
  }

  void sendNavigation()
  {
    const bool fresh = last_cmd_vel_ != std::chrono::steady_clock::time_point{} &&
      std::chrono::duration<double>(std::chrono::steady_clock::now() - last_cmd_vel_).count() <= cmd_vel_timeout_;
    writeFrame(decision_node::mcu::makeNavigationFrame(nav_sequence_++, heroes_never_die_, fresh ? vx_ : 0.0F,
                                                        fresh ? vy_ : 0.0F, fresh ? wz_ : 0.0F));
  }

  void sendMotion() { writeFrame(decision_node::mcu::makeMotionFrame(motion_mode_, 0, bullet_up_, bullet_num_)); }

  static void publishValue(const rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr& publisher, const uint8_t value)
  { std_msgs::msg::UInt8 message; message.data = value; publisher->publish(message); }

  static void publishValue(const rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr& publisher, const uint16_t value)
  { std_msgs::msg::UInt16 message; message.data = value; publisher->publish(message); }

  void publishPoint(const rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr& publisher, const double x, const double y)
  { geometry_msgs::msg::Point point; point.x = x; point.y = y; publisher->publish(point); }

  void publish(const decision_node::mcu::DecodedGameData& d)
  {
    publishValue(game_progress_, d.game_progress); publishValue(remain_hp_, d.self_hp); publishValue(bullet_remain_, d.bullet_remain);
    publishValue(occupy_status_, d.occupy_status); publishValue(robot_id_, d.robot_id); publishValue(robot_color_, d.robot_color);
    publishValue(self_hp_, d.self_hp); publishValue(self_max_hp_, d.self_max_hp);
    publishValue(red_1_hp_, d.red_1_hp); publishValue(red_3_hp_, d.red_3_hp); publishValue(red_7_hp_, d.red_7_hp); publishValue(red_dead_, d.red_dead_bits);
    publishValue(blue_1_hp_, d.blue_1_hp); publishValue(blue_3_hp_, d.blue_3_hp); publishValue(blue_7_hp_, d.blue_7_hp); publishValue(blue_dead_, d.blue_dead_bits);
    publishPoint(enemy_hero_, d.enemy_hero_x, d.enemy_hero_y); publishPoint(enemy_engineer_, d.enemy_engineer_x, d.enemy_engineer_y);
    publishPoint(enemy_std3_, d.enemy_std3_x, d.enemy_std3_y); publishPoint(enemy_std4_, d.enemy_std4_x, d.enemy_std4_y); publishPoint(enemy_sentry_, d.enemy_sentry_x, d.enemy_sentry_y);
    publishValue(suggested_target_, d.suggested_target); publishValue(radar_flags_, d.radar_flags);
    publishValue(can_free_revive_, d.can_free_revive); publishValue(can_instant_revive_, d.can_instant_revive); publishValue(hurt_info_, d.hurt_info);
    geometry_msgs::msg::Point input; input.x = d.operator_x; input.y = d.operator_y; operator_input_->publish(input);
  }

  std::string serial_port_; int baudrate_{}; double nav_frequency_{}, cmd_vel_timeout_{}, reconnect_interval_{}; int fd_{-1};
  std::chrono::steady_clock::time_point last_open_attempt_{}, last_cmd_vel_{}; uint8_t nav_sequence_{}, heroes_never_die_{}, motion_mode_{}, bullet_up_{}, bullet_num_{};
  float vx_{}, vy_{}, wz_{}; decision_node::mcu::GameFrameParser parser_;
  rclcpp::TimerBase::SharedPtr receive_timer_, navigation_timer_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr motion_subscription_, recover_subscription_, bullet_up_subscription_, bullet_num_subscription_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr game_progress_, occupy_status_, robot_id_, robot_color_, suggested_target_, can_free_revive_, can_instant_revive_, hurt_info_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr remain_hp_, bullet_remain_, self_hp_, self_max_hp_, red_1_hp_, red_3_hp_, red_7_hp_, blue_1_hp_, blue_3_hp_, blue_7_hp_, red_dead_, blue_dead_, radar_flags_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr enemy_hero_, enemy_engineer_, enemy_std3_, enemy_std4_, enemy_sentry_, operator_input_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<McuCommunicator>()); rclcpp::shutdown(); return 0;
}

// 此文件用于提供 ROS2 双 MID360 点云处理节点的进程入口。
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "livox_cloudpoint_processor/cloud_processor.hpp"

// 此函数用于初始化 ROS2、运行点云处理节点并在退出时释放资源；输入为命令行参数，输出为进程退出码。
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<livox_cloudpoint_processor::CloudProcessor>());
  rclcpp::shutdown();
  return 0;
}
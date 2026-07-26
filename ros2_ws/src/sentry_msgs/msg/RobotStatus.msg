# 此文件用于定义裁判系统发布的单个比赛机器人状态。
std_msgs/Header header

uint8 id
uint8 level
uint16 remain_hp
uint16 max_hp
uint16 shooter_barrel_cooling_value
uint16 shooter_barrel_heat_limit
uint16 chassis_power_limit
bool gimbal_output
bool chassis_output
bool shooter_output

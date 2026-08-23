# 此文件用于描述 HDL 扫描匹配的状态消息。
std_msgs/Header header

float32 matching_error
float32 inlier_fraction
geometry_msgs/Transform relative_pose

std_msgs/String[] prediction_labels
geometry_msgs/Transform[] prediction_errors

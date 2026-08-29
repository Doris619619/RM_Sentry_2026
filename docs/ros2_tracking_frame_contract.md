# ROS2 Tracking Frame Contract

This contract fixes the coordinate semantics before the ROS2 tracking adapter is wired.
No Global Planning algorithm, trajectory message schema, or `map -> odom -> base_link`
TF topology is changed.

| Data | ROS1 observed semantics | ROS2 semantics | Conversion owner |
| --- | --- | --- | --- |
| `/localization/odometry.header.frame_id` | world/map-equivalent localization frame | `map` | HDL localization |
| `/localization/odometry.child_frame_id` | `aft_mapped`, consumed as LiDAR/gimbal body | `base_link` | HDL localization |
| Odometry pose | world pose of LiDAR/robot | `T_map_base` | HDL localization |
| Odometry twist | Tracking treats it as body/gimbal velocity | `base_link` twist | HDL from Point-LIO odometry |
| `/global_trajectory` | global-map coordinates | `map`; message has no frame field | no conversion |
| `/filted_topic_3d` | filtered local scan | `base_link` | HDL only; not Tracking input |
| `/aligned_points` | ROS1 `/aligned_points_local` world cloud | `map` cloud | HDL localization |
| MPC state `[x,y,v,phi]` | world coordinates | `map` | Tracking Core |
| MPC output `v,phi` | world velocity magnitude and direction | `map` | Tracking Core |
| `/sentry_des_speed` vx/vy | world velocity rotated into gimbal/body | `base_link` vx/vy | Tracking, exactly once |
| `/cmd_vel` | MCU consumes body/gimbal velocity | `base_link` vx/vy | hit_bridge has no rotation |
| Pose yaw | world-to-LiDAR/gimbal yaw | `map -> base_link` yaw | odometry quaternion |
| Velocity heading | world velocity heading | `map` | rotate body twist into map once |
| Wheel/chassis yaw | side-channel input not used by primary control | retained but not mixed with pose yaw | no conversion |
| Independent gimbal yaw | comments only, no formal input | `base_link` orientation is used for now | hardware validation deferred |

Point clouds not expressed in `map` may be transformed with tf2 only. If that transform
is unavailable, Tracking drops the cloud and emits a throttled warning; it must not insert
body-frame points into a world-frame occupancy grid.

The HDL velocity fill-in must obtain Point-LIO's world velocity from the same cached odometry
sample used for localization. It rotates linear velocity with `R_odom_base^T` into
`base_link`, rotates the 3x3 linear covariance the same way, forwards angular velocity
under Point-LIO's body-frame convention, and publishes zero twist plus high covariance if the
sample is absent or older than 0.2 seconds.

# ROS2 Decision / MCU contract

## Source of truth

- ROS1 decision source: `DecisionNode/src/decision_node/src/strategy_node.cpp`, `central_occupiable.cpp`, `motion_change.cpp`, `recover_change.cpp`, `chase.cpp`, and `config/strategy_tree.xml`.
- ROS1 MCU source: `mcu_comm.hpp` and `mcu_communicator.cpp`. Historical `*.backup`, README protocol descriptions, `command_test`, and `test.cpp` are not runtime sources of truth.
- The ROS2 Planning boundary remains `geometry_msgs/msg/PointStamped` on `/clicked_point`; Tracking owns `/global_trajectory -> /sentry_des_speed -> /cmd_vel` and returns `/dstar_status`.

## Preserved behavior

- BehaviorTree.CPP v3 is retained. `BTCPP_format="4"` in the legacy XML is parsed by the installed v3 library; no state-machine rewrite was made.
- The tree, actions, 20 Hz default tick, two-second harm history, harm hysteresis, five-second motion cooldown for transitions inside `{0,1,2}`, no-op vision/timer nodes, chase target mapping, 1 cm chase de-duplication, and `/motion`, `/recover`, `/bullet_up`, `/bullet_num` topics are carried forward.
- `recover` maps to `heroes_never_die` in the recurring navigation frame. It does not set `MotionCommandFrame.hp_up`.
- RX enemy positions are `int16 cm / 100.0 -> metres`. The legacy `-8888.0` Decision sentinel is preserved even though the raw protocol unit is unresolved.

## Deliberate safety improvements

- The stream parser validates SOF, 78-byte header length, game packet type, header CRC8, packet CRC16, and trailer before publishing. It handles partial reads, concatenated frames, garbage, and `H H K` resynchronisation.
- Navigation TX has a configurable watchdog (default 0.5 s). Stale `/cmd_vel` is encoded as zero `vx/vy/wz`, while frames continue at 50 Hz.
- The serial transport is non-blocking, retries open at a configurable two-second interval, resets the parser after I/O failure, and has no receive thread to deadlock during shutdown.

## Historical incompatibilities retained deliberately

- The XML passes `occupy_threshold`, but ROS1 reads `~central_threshold` in the accumulator (default 20). ROS2 exposes both and uses only `central_threshold` for that node.
- No formal publisher of `/referee/friendly_score` or `/referee/enemy_score` exists in source. Both inputs remain optional and default to zero; no score is inferred from HP, death bits, or game result.
- Legacy standalone launches default to 115200, but the integrated launch and HK protocol comment use 921600. ROS2 production MCU config defaults to 921600; 115200 remains a parameter override.

## Deferred hardware validation

The software tests do not prove `/dev/ttyUSB*` electrical stability, real 921600 baud operation, actual MCU interpretation of 21-byte navigation or 7-byte motion frames, referee/radar field truth, enemy sentinel units, revive behavior, chassis response, or long-duration serial operation.

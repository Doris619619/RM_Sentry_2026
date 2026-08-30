# Hardware and system acceptance: Decision / MCU closure

This document is the deployment gate for the ROS2 Decision and MCU bridge. It does not change Decision, Planning, Tracking, OCS2, or the HK packet contract.

## Score-driven RADICAL branch

There is no formal source for `/referee/friendly_score` or `/referee/enemy_score` in the 78-byte HK game frame or the ROS2 MCU communicator. Both inputs remain optional and default to zero. Do not derive them from HP, death bits, winner, kill count, or other referee data.

Before a match, record one accepted source of truth:

1. a specified new field in the MCU HK protocol;
2. a named ROS node with a documented source; or
3. retirement of the score-driven strategy branch.

## Hardware acceptance still required

Software PTY evidence covers a real Linux pseudo-terminal at 921600 baud, raw 8N1, disabled hardware/software flow control, 78-byte RX, and 21/7-byte TX framing. It does not prove real `/dev/ttyUSB*` electrical stability, a real MCU, real referee data, chassis response, radar/enemy truth, or long-duration serial operation.

Before deployment, validate those items with the exact production UART adapter, MCU firmware, referee system, chassis, and field hardware. Record serial error/reconnect behavior during a sustained run.

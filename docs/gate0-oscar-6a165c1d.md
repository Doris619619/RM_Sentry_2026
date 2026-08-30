# Gate 0: Oscar ROS1 source baseline

The ROS1 source-of-truth is `OscarXuHz/RM_Sentry_2026` commit
`6a165c1d91fb6c5c1bb2526ac99d60f819bf51cd`. This record came from a sparse,
detached checkout of that commit.

## Tree comparison

| Package path | Oscar tree | Doris tree | Decision |
| --- | --- | --- | --- |
| `sim_nav/src/hdl_global_localization` | `dcb25fe3646e99d1320cea14eb5dbf2352800adc` | same | Port the Oscar behavior. |
| `sim_nav/src/hdl_localization` | `66e7684e9263f96e101fa63970e9558685813966` | same | Preserve optional global localization. |
| `DecisionNode` | `cf7435626dea81f2ece7a6d8f3de1ab7dec8dfa1` | same | Debug executables are non-production. |
| `sentry_planning/rviz_plugins` | `d3b835b19b003b409bfc3bc933f1f5a7d19cd671` | same | Goal3DTool/Pose3DTool only. |
| `sentry_planning/trajectory_generation` | `b5253cb06ab49f79c27c4d05db4af47eb62cfec8` | `728c9ec5443a63c08305a8205abdee4f2d7650d5` | Oscar is the behavior baseline. |
| `sentry_planning/trajectory_tracking` | `810f9e84da2ec17b8a413ba309e4b48ca2ad6225` | `c8bdfb4c5c2a37d42f71a454b35b58fd37d5639b` | Oscar is the behavior baseline. |

## Defaults retained by the ROS2 port

* Standard launch loads `general_config.yaml`: `BBS`.
* Bare node fallback: `FPFH_RANSAC`.

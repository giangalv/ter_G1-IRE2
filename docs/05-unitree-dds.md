# 05 — Unitree native topics (DDS domain 0)

Seen from the laptop with the environment in [01](01-network-and-computers.md).
This is the robot's own stack, independent of the hand protocol.

| Topic | Type | Notes |
|---|---|---|
| `/lowstate` | `unitree_hg/LowState` | whole-body joint state |
| `/lowcmd` | `unitree_hg/LowCmd` | low-level joint commands (dangerous) |
| `/arm_sdk` | `unitree_hg/LowCmd` | arm-only command interface |
| `/hand_sdk` | `unitree_go/MotorCmds` | 1 subscriber, 0 publishers when checked |
| `/dex3/{left,right}/{cmd,state}` (+ `/lf/dex3/*/state`, `/api/dex3_msg_controller/*`) | `unitree_hg/HandCmd`, `HandState` | Unitree Dex3 hand interface, present in the standard stack; nothing indicates that the Inspire hands use it |
| `/lowstate_doubleimu` | `unitree_hg_doubleimu/doubleIMUState` | IMU state |
| `/unitree/slam_mapping/odom` | `nav_msgs/Odometry` | SLAM odometry (no data while SLAM is off) |
| `/slam_info`, `/slam_key_info` | `std_msgs/String` | event-only SLAM status |
| `/api/<service>/{request,response}` | `unitree_api/Request`, `Response` | RPC to Unitree services (e.g. `slam_operate`) |
| `/unitree/slam_mapping/points`, `/unitree/slam_relocation/points` | `sensor_msgs/PointCloud2` | no publisher at the time |
| `/utlidar/range_info` | — | no publisher at the time |

Nothing in this namespace carries the Inspire hands: they are reached directly
over Modbus TCP ([02](02-inspire-rh56e2-hands.md)).

## `unitree_api` message shape

```
Request:  RequestHeader header { identity {id, api_id}, lease {id}, policy {priority, noreply} }
          string parameter
          uint8[] binary
Response: ResponseHeader header { identity, status {code} }
          string data
          int8[] binary
```

SLAM service ids from a third-party mirror of the docs (**unverified**):
`1801` start mapping (`{"data":{"slam_type":"indoor"}}`), `1802` end mapping,
`1804` init pose, `1102` pose navigation, `1201` pause, `1202` resume,
`1901` close SLAM.

Event-only topics such as `/servicestate`, `/slam_info`, `/slam_key_info`
print nothing on `ros2 topic echo` until something happens.

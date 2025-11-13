# LCM to ROS 2 Bridge

C++ executable that bridges LCM messages from `franka_driver` to ROS 2 topics.

## Overview

This bridge subscribes to the `PANDA_STATUS` LCM channel (drake::lcmt_panda_status) and republishes the robot state as ROS 2 `sensor_msgs/JointState` messages. It integrates seamlessly with the existing codebase without requiring any modifications to `franka_driver.cc`.

## Features

- ✅ Native C++ implementation using existing Drake LCM infrastructure
- ✅ ROS 2 rclcpp node for publishing joint states
- ✅ Configurable via ROS 2 parameters
- ✅ Low latency (1ms polling rate)
- ✅ Zero modifications to existing driver code
- ✅ Integrated with Bazel build system

## Building

Build with Bazel:

```bash
cd /home/yaashiagautam/drake-franka-driver
bazel build //franka-driver:lcm_ros2_bridge
```

The binary will be at: `bazel-bin/franka-driver/lcm_ros2_bridge`

## Usage

### Basic Usage

**Terminal 1: Run the franka driver**
```bash
./bazel-bin/franka-driver/franka_driver_v5 \
  --robot_ip_address=<ROBOT_IP> \
  --control_mode=position
```

**Terminal 2: Run the bridge**
```bash
# Set RMW middleware and run bridge via bazel
cd /home/yaashiagautam/drake-franka-driver
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bazel run //franka-driver:lcm_ros2_bridge
```

**Terminal 3: Verify ROS topics**
```bash
# Must cd into ros_humble directory for setup scripts to work
cd /home/yaashiagautam/drake-franka-driver/ros_humble
bash -c "source setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 topic list"
bash -c "source setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 topic echo /franka/joint_states"
```

> **Note**: Run the bridge with `bazel run` instead of directly executing the binary. This ensures Bazel sets up the runtime library paths correctly for ROS 2 shared libraries.

### Advanced: Custom Parameters

Run with custom LCM URL or topic names:

```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
bazel run //franka-driver:lcm_ros2_bridge -- \
  --ros-args \
  -p lcm_url:="udpm://239.255.76.67:7667?ttl=0" \
  -p lcm_channel:="PANDA_STATUS" \
  -p ros_topic:="/robot/joint_states"
```

> **Note**: The `--` separator is required to pass arguments to the binary when using `bazel run`.

## Parameters

| Parameter     | Type   | Default            | Description                      |
|---------------|--------|--------------------|----------------------------------|
| `lcm_url`     | string | "" (auto)          | LCM URL (empty = default udpm)   |
| `lcm_channel` | string | "PANDA_STATUS"     | LCM channel to subscribe         |
| `ros_topic`   | string | "/franka/joint_states" | ROS topic to publish        |

## Message Mapping

### Input: drake::lcmt_panda_status (LCM)

- `utime` - Timestamp (microseconds)
- `num_joints` - Number of joints (7 for Panda)
- `joint_position` - Joint positions (rad)
- `joint_velocity` - Joint velocities (rad/s)
- `joint_torque` - Joint torques (Nm)
- `joint_position_desired` - Desired positions
- `joint_velocity_desired` - Desired velocities
- `joint_acceleration_desired` - Desired accelerations
- `joint_torque_desired` - Desired torques
- `joint_torque_external` - External torques
- `robot_mode` - Robot operational mode
- `control_command_success_rate` - Command success rate

### Output: sensor_msgs/JointState (ROS 2)

- `header.stamp` - ROS timestamp
- `header.frame_id` - "panda_link0"
- `name` - Joint names (panda_joint1-7)
- `position` - Joint positions (from joint_position)
- `velocity` - Joint velocities (from joint_velocity)
- `effort` - Joint torques (from joint_torque)

## Architecture

```
┌─────────────────┐         LCM          ┌──────────────────┐
│ franka_driver   ├────────────────────► │ lcm_ros2_bridge  │
│ (existing code) │  PANDA_STATUS        │  (new binary)    │
└─────────────────┘  drake::lcmt_panda   └────────┬─────────┘
                          _status                  │
                                                   │ ROS 2
                                                   │
                                           ┌───────▼────────┐
                                           │  /franka/      │
                                           │  joint_states  │
                                           └────────────────┘
                                             sensor_msgs::
                                             JointState
```

## Troubleshooting

### Bridge doesn't receive LCM messages

Check LCM is working:
```bash
lcm-spy  # Should show PANDA_STATUS messages
```

Verify multicast route:
```bash
route -n | grep 239.255.76.67
```

### ROS topics not visible

The bundled `ros_humble/setup.bash` requires being run from within the `ros_humble` directory. Use this pattern:

```bash
cd /home/yaashiagautam/drake-franka-driver/ros_humble
bash -c "source setup.bash && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && ros2 topic list"
```

Alternatively, if you have ROS 2 Humble installed system-wide:
```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic list
```

### Build errors

**Missing `ros2_sensor_msgs` repository**: The correct dependency is `@ros2_common_interfaces//:cpp_sensor_msgs`, not `@ros2_sensor_msgs`. This is already configured in the `BUILD.bazel`.

Ensure ROS 2 Bazel rules are configured. Check `MODULE.bazel` for:
- `@ros2_rclcpp`
- `@ros2_common_interfaces`

### Runtime library errors

If you get `dlopen error: ... cannot open shared object file`, use `bazel run` instead of directly executing the binary:

```bash
# ✅ CORRECT - Bazel sets up library paths
bazel run //franka-driver:lcm_ros2_bridge

# ❌ WRONG - Missing runtime library paths
./bazel-bin/franka-driver/lcm_ros2_bridge
```

## Future Enhancements

- [ ] Add geometry_msgs/WrenchStamped publisher for external wrench
- [ ] Add geometry_msgs/PoseStamped publisher for end-effector pose
- [ ] Add bidirectional bridge (ROS → LCM for commands)
- [ ] Add tf2 transforms for robot links
- [ ] Add diagnostic messages for robot mode/status

## Dependencies

- Drake (with LCM types)
- LCM C++ library
- ROS 2 Humble rclcpp
- ROS 2 sensor_msgs

## License

Same as drake-franka-driver parent project.


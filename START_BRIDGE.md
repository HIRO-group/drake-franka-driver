# Start Extended C++ LCM-ROS2 Bridge

## Quick Start

### Terminal 1: Run Franka Driver (if not already running)
```bash
cd ~/drake-franka-driver
./bazel-bin/franka-driver/franka_driver_v5 --robot_ip_address=192.168.0.2
```

### Terminal 2: Run Extended C++ Bridge
```bash
cd ~/drake-franka-driver
source ros_humble/setup.zsh
bazel run //franka-driver:lcm_ros2_bridge
```

**Expected output:**
```
[INFO] LCM to ROS 2 Bridge initialized
[INFO]   LCM URL: default (udpm://239.255.76.67:7667)
[INFO]   LCM channel: PANDA_STATUS
[INFO] Publishing to ROS topics:
[INFO]   - /franka/joint_states (measured)
[INFO]   - /franka/joint_states_desired
[INFO]   - /franka/external_torques
[INFO]   - /franka/robot_status
```

### Terminal 3: Verify All 4 Topics
```bash
source ~/drake-franka-driver/ros_humble/setup.zsh

# List all Franka topics
ros2 topic list | grep franka
```

**Expected:**
```
/franka/joint_states
/franka/joint_states_desired
/franka/external_torques
/franka/robot_status
```

---

## View Data

### View measured joint states:
```bash
ros2 topic echo /franka/joint_states
```

### View desired joint states:
```bash
ros2 topic echo /franka/joint_states_desired
```

### View external torques (collision detection):
```bash
ros2 topic echo /franka/external_torques
```

### View robot status:
```bash
ros2 topic echo /franka/robot_status
```

---

## Check Publishing Rate

```bash
# Should be ~1000 Hz
ros2 topic hz /franka/joint_states
```

---

## Compare Measured vs Desired (Tracking Error)

### Terminal A:
```bash
ros2 topic echo /franka/joint_states --field position
```

### Terminal B:
```bash
ros2 topic echo /franka/joint_states_desired --field position
```

---

## Monitor External Torques (Collision Detection)

```bash
# Watch for spikes indicating external forces
ros2 topic echo /franka/external_torques --field effort
```

---

## Record All Data

```bash
ros2 bag record /franka/joint_states /franka/joint_states_desired /franka/external_torques /franka/robot_status
```

---

## Visualization

### Plot joint positions:
```bash
rqt_plot /franka/joint_states/position[0]:position[1]:position[2]
```

### Plot external torques:
```bash
rqt_plot /franka/external_torques/effort[0]:effort[1]:effort[2]
```

---

## Troubleshooting

### Bridge doesn't start:
```bash
# Check if LCM is publishing
lcm-spy

# Rebuild if needed
cd ~/drake-franka-driver
bazel build //franka-driver:lcm_ros2_bridge
```

### Only 1 topic appears:
```bash
# Kill old bridge and restart
pkill -f "lcm_ros2_bridge"
bazel run //franka-driver:lcm_ros2_bridge
```

### No topics appear:
```bash
# Check ROS 2 is sourced
source ~/drake-franka-driver/ros_humble/setup.zsh

# Check if bridge is running
ros2 node list | grep lcm_ros2_bridge
```

---

## Summary

**The extended C++ bridge now publishes:**

1. ✅ `/franka/joint_states` - Measured positions, velocities, torques
2. ✅ `/franka/joint_states_desired` - Desired states from controller
3. ✅ `/franka/external_torques` - External torques (collision detection)
4. ✅ `/franka/robot_status` - Control success rate, robot mode

**Fast, efficient, and built with Bazel!** 🚀


# UMAIR (DOCUMENTATION IN PROGRESS)

Running the driver
```bash
./bazel-bin/franka-driver/franka_driver_v5 --robot_ip_address=192.168.0.2
```

Running LCM:
```bash
bazelisk run //franka-driver:lcm_ros2_bridge -- --use_mbp --expire_sec=0.05 --robot_ip_address=192.168.0.2 --control_mode=position --use_torque_for_position=true --vacuum
```
Checking state:
```bash
ros2 topic echo /franka/joint_states
```

Build:
```bash
bazelisk build //...
```

# Subscriber that writes to shared memory:
Run it via:
```bash
bazel run //franka-driver:sdf_subscriber 
```

## Cluster ESDF Subscriber & Shared Memory Interface

This ROS2 node subscribes to `/nvblox/esdf_results` and writes the parsed cluster data to **Boost.Interprocess shared memory** for consumption by other processes (e.g., PandaDriver).

### Shared Memory Segment

* **Segment Name:** `MySharedMemory`
* **Object Name:** `SharedData`
* **Type:** `SharedMemoryData`

### Data Written

All cluster data is written into the `clusters` vector in **flattened numeric format**.

**Layout per cluster (11 doubles):**

| Index | Description                        | Type   |
| ----- | ---------------------------------- | ------ |
| 0     | Cluster label                      | double |
| 1-3   | Centroid (x, y, z)                 | double |
| 4-6   | Capsule start point `p0` (x, y, z) | double |
| 7-9   | Capsule end point `p1` (x, y, z)   | double |
| 10    | Capsule radius                     | double |

Multiple clusters are stored sequentially:

```
[label, cx, cy, cz, p0x, p0y, p0z, p1x, p1y, p1z, radius, ... next cluster ...]
```

### Flags

* `clusters_ready` is set to `true` whenever new data is written.
* Other fields (`data`, `ee_wrench`, `gripper_cmd`) remain unchanged.

### Notes

* Access to `clusters` is protected by `mutex` for thread/process safety.
* The subscriber **overwrites the cluster data on each message**, ensuring the latest state is always available.

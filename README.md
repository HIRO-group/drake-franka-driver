# Panda Arm - Franka Emika
This repository contains the application code used to communicate with the Franka Emika arm from Drake. 

## Building the driver
Before you begin, install the prerequisite:
```
sudo apt install libgflags-dev
```

To build, run `bazel build //...`.  This will output two versions of
the driver: `bazel-bin/franka-driver/franka_driver_v4` and
`bazel-bin/franka-driver/franka_driver_v5`.

Use `franka_driver_v5` for the FR-3 robots.

# Running the driver

You can run the driver via `bazel run` (recommended) or using the built binary in `bazel-bin`.

When using `bazel run`, remember to separate Bazel flags from program flags with `--`.

## Common commands

Run the v4 driver (Franka Panda) in position mode using a torque-based position controller and vacuum gripper:

```
bazelisk run //franka-driver:franka_driver_v4 -- \
  --use_mbp \
  --expire_sec=0.05 \
  --robot_ip_address=192.168.0.2 \
  --control_mode=position \
  --use_torque_for_position=true \
  --vacuum
```
> You can use the V5 Driver if using the FR3

Start the status subscriber / shared-memory bridge:

```
bazelisk run //franka-driver:panda_status_drake_subscriber_main
```

Optional verbose debug output for the subscriber:

```
bazelisk run //franka-driver:panda_status_drake_subscriber_main -- --debug
```

## Flag reference (driver)

- `--robot_ip_address=STRING`: IP address of the robot controller (required).
- `--control_mode=STRING`: One of `status_only`, `velocity`, `position`, `torque`.
- `--expire_sec=DOUBLE`: Max command staleness allowed before rejecting commands.
- `--use_mbp`: Load a Drake MultibodyPlant model for internal dynamics computations.
- `--use_torque_for_position`: In `position` mode, use a torque controller to track positions.
- `--vacuum` (alias `--vacumm`): Use the vacuum gripper instead of the parallel gripper.
- `--gripper_ip_address=STRING`: Override the IP for the gripper device (defaults to robot IP).
- `--lcm_gripper_command_channel=STRING`: LCM channel for gripper/vacuum commands (default `PANDA_GRIPPER_COMMAND`).

## Flag reference (subscriber)

- `--debug`: Enable verbose prints for the Panda status subscriber and shared-memory writes.

## Notes on binaries

- `franka_driver_v4` should be used with FE-3 firmware (libfranka v0.8.x).
- `franka_driver_v5` should be used with FR-3 firmware (libfranka v0.10.x).

To run using the built binary directly (without `bazel run`):

```
bazel-bin/franka-driver/franka_driver_v5 --expire_sec=0.05 --robot_ip_address=<ip-addr> --control_mode=position
```

To add a gripper model to a simulation or directives file, update the relevant model directives.

## Links

* [Website](https://www.franka.de/technology)
* [Franka Control Interface (FCI) Documentation](https://frankaemika.github.io/docs/)
* [`libfranka` API Docs](https://frankaemika.github.io/libfranka/)
* [Panda datasheet](https://s3-eu-central-1.amazonaws.com/franka-de-uploads/uploads/Datasheet-EN.pdf)

## Notes

* Be sure to read the manual, FCI docs, etc.
    * The FCI docs have excellent instructions for getting your network
    interface on Ubuntu set up.
* There are two very distinct versions of Pandas: the FE3 (older) and FR3 (newer)
* There are three version to think about: firmware on the robot (we use
`v4.x`, and `v5.x`) and `libfranka` driver software on the host PC
(`0.8.0` for `v4.x` FE3, and `0.10.0` for `v5.x` FR3).
    * You should see suffixes like `_v4` and `_v5` for relevant
    libraries and binaries.
    * If you use the wrong driver, you should get an error like:

            terminate called after throwing an instance of 'franka::IncompatibleVersionException'
            what():  libfranka: Incompatible library version (server version: 4, library version: 5). Please check https://frankaemika.github.io for Panda system updates or use a different version of libfranka.


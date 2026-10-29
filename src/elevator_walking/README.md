# elevator_walking

This package provides `move_relative`, a command-line wrapper for Unitree SDK 2's
Go2 `ObstaclesAvoidClient.MoveToIncrementPosition` API. It sends an offset from
the robot's current position, so it does not need a prebuilt map. The robot must
expose and support the `obstacles_avoid` service. This CLI does not toggle the
robot's obstacle-avoidance or control-source settings. The service's sensor
requirements depend on the robot firmware and are not verified here.

The x/y arguments use meter-scale offsets (`1.0` means 1 m = 100 cm); `yaw` is
in radians. The upstream API names the relative-pose fields but does not state
their units in the source headers, so this CLI assumes standard SI units. Confirm
this against the target robot's firmware before physical use. The official
client sends this API without a completion response, so the command's return
code confirms submission only, not arrival at the target.

Install the official `unitree_sdk2_python` package and its CycloneDDS dependency
before running. The SDK is not included in this workspace. See the
[official installation guide](https://github.com/unitreerobotics/unitree_sdk2_python#installation).

Build and run from the workspace root:

```bash
source /opt/ros/foxy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 run elevator_walking move_relative enp3s0 --x 1.0
```

Replace `enp3s0` with the network interface connected to the Go2. A lateral
offset can be supplied with `--y`; a heading offset can be supplied with `--yaw`.

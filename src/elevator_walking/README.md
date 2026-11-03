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

`publish_lidar_distance` subscribes to `/utlidar/cloud`, selects points within
10 degrees horizontally and 15 degrees vertically of the forward x-axis, and
publishes their median x-coordinate as meters on `/utlidar/front_distance` at
10 Hz, aggregating points across a rolling 0.3-second window to tolerate sparse
individual scans. Stale input or too few points in the full window produce
`NaN`. This estimate is the median within the forward cone, not the closest
point in the whole cloud.

`move_until_distance` selects its distance input with the `distance_source`
parameter: `lidar` (default) reads `/utlidar/front_distance`, and `realsense`
reads `/camera/depth/center_distance`. It repeatedly sends a forward `Move`
command until the distance falls below `threshold_m`, then sends `StopMove`. It
also stops if distance becomes invalid or stale, or when `max_motion_s` is
reached. Its default `vx` is a cautious `0.01`.

`move_relative` requires the official `unitree_sdk2_python` package and its
CycloneDDS dependency; that SDK is not included in this workspace. See the
[official installation guide](https://github.com/unitreerobotics/unitree_sdk2_python#installation).
The lidar distance and `move_until_distance` nodes use the ROS 2 `unitree_api`
messages and do not use the Python SDK.

`move_forward_distance` publishes a `Move` request to `/api/sport/request` every
0.1 seconds by default, subscribes to the ROS `/lowstate` topic, and publishes
`StopMove` when wheel `q` estimates the requested travel. It uses Go2-W wheel
motor indices 12–15 and estimates travel as wheel angle change times the
configured radius (default `0.0889 m`). This is an estimate of wheel travel,
not guaranteed body displacement; tire compression, wheel slip, and stopping
delay affect accuracy. It uses the ROS `unitree_api` and `unitree_go` messages;
it does not use the Python SDK.

Build and run from the workspace root:

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w/dlio_ws/install_foxy/setup.bash
cd /home/unitree/elevator-walking
colcon build --packages-select elevator_walking --symlink-install
source install/setup.bash
ros2 run elevator_walking move_relative enp3s0 --x 1.0
```

Replace `enp3s0` with the network interface connected to the Go2. A lateral
offset can be supplied with `--y`; a heading offset can be supplied with `--yaw`.

To move forward by an estimated wheel distance, run:

```bash
ros2 run elevator_walking move_forward_distance --ros-args \
  -p distance_m:=0.20 -p vx:=0.1
```

The command stops on the requested wheel-angle estimate, missing `LowState`,
wheel-angle stall, maximum run time, or Ctrl-C. Start with a short distance in
an open area and compare the reported estimate with measured floor travel; set
`--wheel-radius-m` to a calibrated effective radius if needed.

## View LiDAR in a browser with Foxglove

Use the robot's ROS domain for all three steps below. In this setup that is
usually domain `0`; a separate domain such as `1` will not show the robot's
topics unless the robot is also using that domain.

1. **Start rosbridge on the Jetson.** In a Jetson terminal, source the Foxy and
   Unitree message environments, then leave rosbridge running:

   ```bash
   source /opt/ros/foxy/setup.bash
   source /home/unitree/go2w/dlio_ws/install_foxy/setup.bash
   ros2 launch rosbridge_server rosbridge_websocket_launch.xml
   ```

2. **Forward the WebSocket port from the Mac.** In a Mac terminal, run this
   command and keep the terminal open. It is normal for the command not to
   return to a prompt while the tunnel is active:

   ```bash
   ssh -N -L 9090:127.0.0.1:9090 unitree@192.168.11.14
   ```

   Replace the IP address if the Jetson's address changes.

3. **Connect Foxglove in Chrome.** Choose **Rosbridge WebSocket** and enter
   `ws://localhost:9090`. Add a **3D** panel and enable `/utlidar/cloud`; when
   prompted, use `utlidar_lidar` as the fixed frame. To plot the scalar distance,
   add `/utlidar/front_distance` in a Plot panel.

## Start the RealSense D435i

On the Jetson, source ROS 2 Foxy and the Unitree workspace, then launch the
official RealSense ROS driver with color and depth enabled:

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w/dlio_ws/install_foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_sync:=true align_depth.enable:=true
```

Leave this terminal running while using the camera. Stop any other process that
already opened the D435i (including a previous RealSense publisher) first; two
camera drivers can cause `Device or resource busy`. Press Ctrl-C to stop the
driver.

## Preview vertical-line detection

Run the preview node while the RealSense color stream is active:

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w/dlio_ws/install_foxy/setup.bash
source /home/unitree/elevator-walking/install/setup.bash
ros2 run elevator_walking preview_vertical_lines
```

In Foxglove, add an **Image** panel for
`/camera/color/vertical_lines/compressed`. Keep the original
`/camera/color/image_raw/compressed` in a second panel for comparison. The
preview draws candidate vertical segments, merges nearby segments into line
positions, marks the image center, and reports the line and segment counts.
Red pixels show Canny edges; green indicates a candidate within the center
tolerance, and yellow indicates one outside it. If a shadow edge is missing
from the red overlay, lower `canny_low_threshold` and `canny_high_threshold`;
if it is red but not a candidate line, lower `hough_threshold` or
`min_line_length_px`, or increase `max_line_gap_px` and
`max_angle_from_vertical_deg`. For example, try these permissive preview-only
settings and adjust from the image:

```bash
ros2 run elevator_walking preview_vertical_lines --ros-args \
  -p process_width:=960 \
  -p canny_low_threshold:=15 -p canny_high_threshold:=60 \
  -p hough_threshold:=15 -p min_line_length_px:=25 \
  -p max_line_gap_px:=30 -p max_angle_from_vertical_deg:=15
```

These settings can also pick up texture and other false lines. This node only
visualizes detections and does not move the robot.

## Move left until a vertical line is detected

With the camera stream and ROS `unitree_api` messages available, run:

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w/dlio_ws/install_foxy/setup.bash
source /home/unitree/elevator-walking/install/setup.bash
ros2 run elevator_walking move_left_until_line
```

This node uses the same Canny/Hough detector and parameters as
`preview_vertical_lines`. It waits for a valid color frame, then refreshes the
leftward `Move` request at 10 Hz until at least one vertical line cluster is
within `center_tolerance_px` of the image center, and sends `StopMove`. The
default center band is ±20 image pixels. Two or more centered clusters are not
distinguished. Foxglove can display the annotated image on
`/camera/color/move_left_until_line/compressed`. The node stops if the camera
frames become stale, after 20 seconds of movement by default, or on Ctrl-C.
When a line is found, the node exits successfully after sending `StopMove`.
It starts at `vy:=0.1` and latches to `approach_vy:=0.05` when any candidate
line comes within `slowdown_distance_px:=160` of the image center, then stays at
approach speed until it reaches the center band. This limits slow-speed travel
to the final alignment segment.
`vx` is an optional forward/backward correction and defaults to `0.0`; if the
robot drifts forward during lateral motion, try a small negative value such as
`-p vx:=-0.01` and tune it from the observed motion. Keep the lateral path
clear; detector thresholds affect when it stops.

The robot does not currently publish `/tf` or `/tf_static`, so Foxglove can show
the point cloud in its LiDAR frame but cannot show a TF relationship between
the LiDAR and the robot body.

For either sensor, source the same ROS 2 environment in both terminals. Start
the matching distance publisher first. For LiDAR:

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w/dlio_ws/install_foxy/setup.bash
source /home/unitree/elevator-walking/install/setup.bash
ros2 run elevator_walking publish_lidar_distance
```

Check the distance from a second sourced terminal before starting movement:

```bash
source /opt/ros/foxy/setup.bash
source /home/unitree/go2w/dlio_ws/install_foxy/setup.bash
source /home/unitree/elevator-walking/install/setup.bash
ros2 topic echo /utlidar/front_distance
```

After confirming the LiDAR reading, start movement in that terminal:

```bash
ros2 run elevator_walking move_until_distance --ros-args \
  -p distance_source:=lidar -p threshold_m:=0.7 -p vx:=0.01
```

For RealSense, start its depth publisher instead:

```bash
ros2 run elevator_walking publish_center_distance
```

Then select the matching source when starting movement:

```bash
ros2 run elevator_walking move_until_distance --ros-args \
  -p distance_source:=realsense -p threshold_m:=0.7 -p vx:=0.01
```

# Runtime Guide — smartmicro ROS 2 Radar Driver

This document covers everything needed to build, run, and visualize data from the DRVEGRD 169 radar driver in a containerized ROS 2 Humble environment.

## Workspace Structure

The driver must be built inside a ROS 2 colcon workspace. The `point_cloud_msg_wrapper` dependency is not available as a binary apt package for Humble and must be built from source alongside the driver.

Expected layout:

```
~/ros2_ws/
└── src/
    ├── smartmicro_ros2_radars/   ← this repo
    └── point_cloud_msg_wrapper/  ← cloned from GitLab
```

Clone the dependency into your workspace:

```bash
cd ~/ros2_ws/src
git clone https://gitlab.com/ApexAI/point_cloud_msg_wrapper.git
```

## Building

From the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select point_cloud_msg_wrapper umrr_ros2_driver umrr_ros2_msgs
source install/setup.bash
```

To also build the RViz plugin (required for visualization):

```bash
colcon build --packages-select smart_rviz_plugin
source install/setup.bash
```

> **Note:** Re-run `source install/setup.bash` after every build to pick up updated files.

## Known Configuration Gotcha — `hw_dev_id` Must Not Be `1`

The driver uses `hw_dev_id == 1` internally as a sentinel value meaning "not configured". If your adapter's `hw_dev_id` is set to `1`, the node will throw:

```
std::runtime_error: At least one adapter must be configured.
```

The config at `umrr_ros2_driver/param/radar.params.template.yaml` uses `hw_dev_id: 2`. The sensor's `dev_id` must match this value. **Do not change either back to `1`.**

After any change to `radar.params.template.yaml`, you must rebuild for the updated file to be copied into the install directory:

```bash
colcon build --packages-select umrr_ros2_driver
source install/setup.bash
```

## Launching the Driver

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch umrr_ros2_driver radar.launch.py
```

Successful startup output:

```
[smart_radar]: Data stream services have been received!
[smart_radar]: Radar services are ready.
[smart_radar]: Successfully created PORT publishers for sensor 0
```

## Published Topics

The driver does **not** publish to `/umrr/targets`. The actual topics are:

| Topic | Type | Description |
|---|---|---|
| `/smart_radar/port_targets_0` | `umrr_ros2_msgs/msg/PortTargets` | Radar target data |
| `/smart_radar/port_targetheader_0` | `umrr_ros2_msgs/msg/PortTargetHeader` | Target list metadata |

## Echoing Data from a Container

The driver terminal is blocked. To inspect data, open a new terminal and exec into the same container:

```bash
docker exec -it <container_name> bash
```

Inside the container:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /smart_radar/port_targets_0
```

To find the container name:

```bash
docker ps --format "table {{.Names}}\t{{.Image}}\t{{.Status}}"
```

To inspect the message structure:

```bash
ros2 interface show umrr_ros2_msgs/msg/PortTargets
```

## RViz Visualization

RViz requires a display and cannot run inside a headless container. Run it on the host machine or with X11 forwarding.

**On the host machine:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/smartmicro_ros2_radars/umrr_ros2_driver/config/rviz/smart_plugin.rviz
```

**With X11 forwarding into the container:**

```bash
xhost +local:docker
docker exec -it -e DISPLAY=$DISPLAY <container_name> bash
```

**RViz settings:**

- Set `Fixed Frame` to `umrr` under Global Options (left panel)
- The `smart_rviz_plugin` must be built for the custom displays to load (see Build section above)

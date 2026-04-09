# Ethernet Setup Guide — DRVEGRD 169 (UMRR9F) on ROS 2 Humble

This guide covers setting up the smartmicro ROS 2 driver for a single DRVEGRD 169 radar sensor over Ethernet using the `enp3s0` interface.

## Prerequisites

`point_cloud_msg_wrapper` is not available as a binary apt package for Humble — it must be built from source inside your ROS 2 workspace.

Clone it into your workspace `src` directory:

```bash
cd ~/your_ros2_ws/src
git clone https://gitlab.com/ApexAI/point_cloud_msg_wrapper.git
```

Then resolve its dependencies:

```bash
cd ~/your_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

## Step 1 — Configure the host network interface

Assign a static IP to `enp3s0` so the host can communicate with the sensor (which defaults to `192.168.11.11`):

```bash
sudo ifconfig enp3s0 192.168.11.17 netmask 255.255.255.0
```

> This must be run once per boot. To make it persistent, configure it via `netplan` or `nmcli`.

Verify connectivity:

```bash
ping 192.168.11.11
```

## Step 2 — Verify the sensor config

The driver is configured via `umrr_ros2_driver/param/radar.params.template.yaml`. It has already been updated for your setup:

- **Adapter:** `enp3s0`, port `55555`, `hw_dev_id: 2`
- **Sensor:** `192.168.11.11`, port `55555`, `dev_id: 2`, `id: 100`

> **Important:** `hw_dev_id` and `dev_id` must both be set to the same value and must **not** be `1` — the driver treats `1` as the unconfigured default and will throw an error if it sees that value.
- **Model:** `umrr9f_v2_4_1` (firmware V2.4.0)

If your sensor has a different firmware version, update `model` and the `uif*` fields accordingly:

| Firmware version      | `model`          | `uifmajorv` | `uifminorv` | `uifpatchv` |
|-----------------------|------------------|-------------|-------------|-------------|
| UMRR9F Type 169 V2.4.0 | `umrr9f_v2_4_1` | 2           | 4           | 1           |
| UMRR9F Type 169 V2.2.0 | `umrr9f_v2_2_1` | 2           | 2           | 1           |
| UMRR9F Type 169 V2.0.1 | `umrr9f_v2_1_1` | 2           | 1           | 1           |
| UMRR9F Type 169 V1.3.0 | `umrr9f_v1_1_1` | 1           | 1           | 1           |

## Step 3 — Extract the Smart Access library

This must be done once before building:

```bash
./smart_extract.sh
```

## Step 4 — Build the driver

**Option A — Native build (Humble):**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select point_cloud_msg_wrapper umrr_ros2_driver umrr_ros2_msgs
source install/setup.bash
```

**Option B — Docker build:**

```bash
docker build --build-arg ROS_DISTRO=humble -t umrr-ros:humble .
docker run --rm -v`pwd`:/code umrr-ros:humble colcon build --packages-skip smart_rviz_plugin
```

## Step 5 — Launch the driver

```bash
ros2 launch umrr_ros2_driver radar.launch.py
```

The driver will publish `sensor_msgs::msg::PointCloud2` messages on the `/umrr/targets` topic.

Verify data is flowing:

```bash
ros2 topic echo /umrr/targets
```

## Optional — RViz visualization

From a separate terminal after sourcing the workspace:

```bash
rviz2 -d smartmicro_ros2_radars/umrr_ros2_driver/config/rviz/smart_plugin.rviz
```

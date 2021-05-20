# VIA SDK - ROS2-based architecture for multimedia robots

- Co-developed by VIA & AIR.
- VIA project: <https://via.makerviet.org/>.

## I. Setup development environment

### 1. Native development

**Requirements**

- Ubuntu 20.04
- ROS 2 Foxy
- camera_info_manager
- image_transport

### 2. Using Docker

**Requirements**

- Visual Studio Code
- ROS 2: Foxy/Dashing
- Docker

**Step 1: Clone source code and Open in container - VS Code**

![Open In Container](docs/images/open_in_container.png)

**Step 2: Wait for Docker pull and start your contributions**

![](docs/images/docker_pull_vscode.png)

**ROS 2 Cheatsheet:** <https://github.com/ubuntu-robotics/ros2_cheats_sheet>.

## II. Camera node demo

- Build all

```
colcon build
source install/setup.bash
```

- Launch camera node

```
ros2 launch via_camera_nodes camera_node.launch.py
```

- Start camera stream

```
ros2 service call /camera/start std_srvs/srv/Trigger "{}"
```

- Open image viewer

```
ros2 run rqt_image_view rqt_image_view
```

- Stop camera stream

```
ros2 service call /camera/stop std_srvs/srv/Trigger "{}"
```

## III. Issues

### 1. No ROS node name when using `ros2 node list`:

Restart ROS 2 deamon:

```
bash scripts/restart_ros2_deamon.sh
```

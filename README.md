# WILP Driver Stack

Custom ROS 2 Humble software stack for an F1TENTH/RoboRacer platform built from scratch using generic hardware drivers.

## Features

- ROS 2 Humble
- Bringup package
- Keyboard teleoperation
- Vehicle control node
- VESC integration
- Hokuyo URG LiDAR support

## Project Structure

```text
src/
├── bringup/
└── locomotion/
```

## Requirements

- Ubuntu 22.04
- ROS 2 Humble

## Build

```bash
colcon build
source install/setup.bash
```

## Launch

```bash
ros2 launch bringup bringup.launch.py
ros2 run locomotion keyboard_node
```

## License

MIT

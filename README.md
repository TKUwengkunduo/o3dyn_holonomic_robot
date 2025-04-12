# O3Dyn Holonomic Robot

This is a ROS 2 Humble-based demo project for controlling an omnidirectional robot platform. The main node `basic_motion_demo.py` demonstrates smooth and continuous motion commands: forward, lateral, and infinite rotation. This package is suitable for testing hardware or simulation environments such as NVIDIA Isaac Sim.

![Demo Animation](demo.gif)

---

## Project Structure

```bash
o3dyn_ws/
├── demo.gif
├── README.md
├── src/
│   └── demo/
│       ├── demo/
│       │   ├── basic_motion_demo.py      # Main demo node
│       │   └── __init__.py
│       ├── package.xml
│       ├── resource/
│       │   └── demo
│       ├── setup.cfg
│       ├── setup.py
│       └── test/
└── USD/
    └── o3dyn_ros2.usd                   # Robot model for simulation
```

---

## Installation

### 1. Clone the repository

```bash
git clone https://github.com/TKUwengkunduo/o3dyn_holonomic_robot.git
```

### 2. Build the package

```bash
cd ~/o3dyn_holonomic_robot
colcon build
source install/setup.bash
```

---

## Run the Demo Node

```bash
ros2 run demo basic_motion_demo
```

> Ensure your robot subscribes to the `/cmd_vel` topic using `geometry_msgs/msg/Twist` messages.


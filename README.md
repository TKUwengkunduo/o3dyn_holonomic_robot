# O3dyn Holonomic Robot

This is a ROS 2 Humble-based demo project for controlling an omnidirectional robot platform. The main node `basic_motion_demo.py` demonstrates smooth and continuous motion commands: forward, lateral, and infinite rotation. This package is suitable for testing hardware or simulation environments such as NVIDIA Isaac Sim.

![Demo Animation](demo.gif)

---

## Project Structure

```bash
o3dyn_holonomic_robot/
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
    └── o3dyn_full.usd                    # Robot model for simulation
    └── Gamepad_ogn.usd                   # Graph
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

## Launching in Isaac Sim with ROS2

To simulate and control the robot using ROS2 in Isaac Sim:

1. **Open the USD file in Isaac Sim**
   - Launch Isaac Sim.
   - Copy `USD/o3dyn_full.usd` and `USD/Gamepad_ogn.usd` to `Local nucleus service`, and make sure they are in the same folder. 
   - If you don't know about `Local Nucleus Service`, you can choose to open the file directly. Just make sure they are in the same folder. But there may be no floor, you can create one in `Creat>>Environments>>Flat Grid`
   - Run project.

2. **Verify ROS2 Communication**
   - In a new terminal:

     ```bash
     ros2 topic list
     ```

   - You should see `/cmd_vel` listed, confirming the robot is publishing/subscribing as expected.

---

## Run the Demo Node

This will begin the motion sequence: forward, lateral, and continuous rotation:

```bash
ros2 run demo basic_motion_demo
```

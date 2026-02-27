# GRACE Self-Balancing Robot

A two-wheeled self-balancing robot built with **ROS 2** and controlled using a **Linear Quadratic Regulator (LQR)**. The project includes both **Gazebo simulation** and **real hardware** support via an ESP32 + hoverboard motor platform.

![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Jazzy-blue)
![License](https://img.shields.io/badge/License-Apache_2.0-green)

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware](#hardware)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Simulation](#simulation)
  - [Real Robot](#real-robot)
- [Project Structure](#project-structure)
- [LQR Controller Design](#lqr-controller-design)
- [Tuning Guide](#tuning-guide)
- [Troubleshooting](#troubleshooting)

---

## Overview

GRACE (Ground Robot for Autonomous Control Experiments) is a two-wheeled inverted pendulum robot that balances itself using LQR optimal control. The system uses:

- **IMU sensor** for measuring the robot's pitch angle (tilt)
- **Wheel encoders** for measuring position and velocity
- **LQR controller** that computes optimal torque commands to keep the robot upright
- **Gazebo simulation** for safe development and testing before deploying to hardware

### Key Features

- Full-state LQR balancing with `[θ, θ̇, p, ṗ]` state vector
- Optional **Luenberger observer** for state estimation
- **Encoder noise simulation** for realistic sim-to-real testing
- Automatic hardware/simulation mode switching via `use_sim_time`
- CSV data logging for analysis and gain tuning
- Emergency tilt protection and sensor watchdog (hardware mode)
- Motor imbalance compensation for real hardware

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                    Launch Files                       │
│  balancing_simulation.launch.py  OR                   │
│  balancing_real_robot.launch.py                       │
└──────────────┬───────────────────┬───────────────────┘
               │                   │
    ┌──────────▼──────────┐  ┌─────▼──────────────────┐
    │   Gazebo / ESP32     │  │  lqr_balancing.launch  │
    │   (Plant/Hardware)   │  │  (Controller)          │
    └──────────┬──────────┘  └─────┬──────────────────┘
               │                   │
    ┌──────────▼──────────┐  ┌─────▼──────────────────┐
    │  /joint_states       │  │  LQR Balancing         │
    │  /imu/out            │  │  Controller Node       │
    └──────────┬──────────┘  │                         │
               │              │  Subscribes:            │
    ┌──────────▼──────────┐  │  · /robot_pitch         │
    │  pitch_monitor       │  │  · /joint_states        │
    │  Node                │  │                         │
    │                      │  │  Publishes:             │
    │  /imu/out            │  │  · /simple_effort_      │
    │      ↓               │  │    controller/commands  │
    │  /robot_pitch        │  │    (sim)                │
    └─────────────────────┘  │  · /motor_torque_        │
                              │    commands (hardware)   │
                              └──────────────────────────┘
```

---

## Hardware

| Component | Specification |
|-----------|--------------|
| **Motors** | 2× Hoverboard BLDC motors (hub wheels) |
| **Motor Driver** | Hoverboard mainboard with FOC firmware |
| **Microcontroller** | ESP32 (UART bridge between ROS 2 and motors) |
| **IMU** | MPU6050 / MPU9250 (connected via ESP32) |
| **Compute** | Any Linux computer running ROS 2 |
| **Wheel Radius** | 0.08255 m |
| **Wheel Separation** | 0.45468 m |

### Wiring Overview

```
Linux PC ←── USB/UART ──→ ESP32 ←── UART ──→ Hoverboard Mainboard
                            ↑                        ↓
                           I2C                  Motor L + Motor R
                            ↓
                          IMU
```

---

## Prerequisites

- **Ubuntu 22.04** (Humble) or **Ubuntu 24.04** (Jazzy)
- **ROS 2 Humble** or **ROS 2 Jazzy**
- **Gazebo** (Ignition Fortress for Humble / Harmonic for Jazzy)
- **Python 3.10+**

### Required ROS 2 Packages

```bash
sudo apt install \
  ros-${ROS_DISTRO}-ros2-control \
  ros-${ROS_DISTRO}-ros2-controllers \
  ros-${ROS_DISTRO}-gazebo-ros2-control \
  ros-${ROS_DISTRO}-ros-gz \
  ros-${ROS_DISTRO}-imu-filter-madgwick \
  ros-${ROS_DISTRO}-controller-manager \
  ros-${ROS_DISTRO}-joint-state-broadcaster \
  ros-${ROS_DISTRO}-effort-controllers \
  ros-${ROS_DISTRO}-xacro \
  ros-${ROS_DISTRO}-robot-state-publisher
```

### Python Dependencies

```bash
pip install numpy transforms3d
```

### Hardware Dependency (real robot only)

```bash
sudo apt install libserial-dev
```

---

## Installation

```bash
# 1. Clone the repository
mkdir -p ~/balancing_ws/src
cd ~/balancing_ws/src
git clone https://github.com/<YOUR_USERNAME>/grace-balancing.git .

# 2. Install dependencies
cd ~/balancing_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Build
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Simulation

Launch the simulation environment and controller in two separate terminals:

**Terminal 1 — Start Gazebo + sensors:**

```bash
source ~/balancing_ws/install/setup.bash
ros2 launch grace_bringup balancing_simulation.launch.py
```

**Terminal 2 — Start the LQR controller:**

```bash
source ~/balancing_ws/install/setup.bash
ros2 launch grace_controller lqr_balancing.launch.py use_sim_time:=True
```

#### Simulation Options

| Argument | Default | Description |
|----------|---------|-------------|
| `use_encoder_noise` | `false` | Enable realistic hoverboard encoder noise |

```bash
# With encoder noise simulation
ros2 launch grace_bringup balancing_simulation.launch.py use_encoder_noise:=true
ros2 launch grace_controller lqr_balancing.launch.py use_sim_time:=True use_encoder_noise:=true
```

### Real Robot

**Terminal 1 — Start hardware interface + sensors:**

```bash
source ~/balancing_ws/install/setup.bash
ros2 launch grace_bringup balancing_real_robot.launch.py
```

**Terminal 2 — Start the LQR controller:**

```bash
source ~/balancing_ws/install/setup.bash
ros2 launch grace_controller lqr_balancing.launch.py use_sim_time:=False
```

#### Real Robot Options

| Argument | Default | Description |
|----------|---------|-------------|
| `serial_port` | `/dev/grace_esp32` | Serial port for the ESP32 |
| `use_observer` | `False` | Enable Luenberger state observer |
| `is_balance_only` | `True` | Balance-only mode (no position tracking) |

### Controller Options (both modes)

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `True` | `True` = simulation, `False` = hardware |
| `use_observer` | `False` | Enable Luenberger observer |
| `use_encoder_noise` | `false` | Remap to noisy encoder topic |
| `is_balance_only` | `True` | Balance-only mode |
| `csv_log_path` | `~/balancing_ws/src/LQR_Calculations/balancing_log.csv` | Path to CSV log |

---

## Project Structure

```
balancing_ws/src/
├── grace_bringup/              # Top-level launch files
│   └── launch/
│       ├── balancing_real_robot.launch.py
│       └── balancing_simulation.launch.py
│
├── grace_controller/           # Controller nodes
│   ├── config/
│   │   └── grace_controllers.yaml
│   ├── grace_controller/
│   │   ├── lqr_balancing_controller.py   # Main LQR controller
│   │   ├── pitch_monitor.py              # IMU → pitch angle
│   │   └── hoverboard_encoder_simulator.py  # Encoder noise sim
│   └── launch/
│       └── lqr_balancing.launch.py
│
├── grace_description/          # Robot model (URDF, meshes, worlds)
│   ├── launch/
│   │   ├── gazebo.launch.py
│   │   └── display.launch.py
│   ├── meshes/                 # STL mesh files
│   ├── urdf/                   # URDF/Xacro robot description
│   ├── rviz/                   # RViz config
│   └── worlds/                 # Gazebo world files
│
├── grace_firmware/             # Hardware interface (ros2_control)
│   ├── firmware/               # ESP32 Arduino firmware
│   ├── src/
│   │   ├── grace_interface.cpp          # ros2_control plugin
│   │   └── kt_calibration_node.cpp      # Motor Kt calibration
│   ├── include/
│   └── launch/
│       └── hardware_interface.launch.py
│
├── grace_msg/                  # Custom message definitions
│   └── msg/
│       └── MotorFeedback.msg
│
└── LQR_Calculations/           # Control design & analysis
    ├── CompensatorDesign.py             # LQR gain computation
    ├── visualize_balancing.py           # Plot balancing logs
    ├── grace_self_balancing_lqr_...md   # Full derivation document
    ├── esp32_firmware_parameters.txt
    └── Prompt.txt
```

---

## LQR Controller Design

The robot is modeled as an **inverted pendulum on a wheeled cart**. The state vector is:

```
x = [θ, θ̇, p, ṗ]
```

Where:
- `θ` — Pitch angle (tilt from vertical), radians
- `θ̇` — Angular velocity of tilt
- `p` — Linear position of the wheels, meters  
- `ṗ` — Linear velocity of the wheels

The control law is:

```
u = -K · x
```

Where `K` is the optimal LQR gain matrix computed by solving the continuous-time algebraic Riccati equation (CARE) with appropriately chosen `Q` and `R` weight matrices.

### Gain Matrices

| Parameter | Simulation | Hardware |
|-----------|-----------|----------|
| K_θ | -17.27 | -28.87 |
| K_θ̇ | -4.95 | -31.86 |
| K_p | -0.01 | -0.14 |
| K_ṗ | -0.25 | -1.48 |

The hardware gains are more aggressive because real-world friction and motor response differ from simulation.

### Design Scripts

The `LQR_Calculations/` directory contains:

- **`CompensatorDesign.py`** — Computes LQR gains, observer gains, and system matrices
- **`visualize_balancing.py`** — Plots CSV log data for tuning analysis
- **`grace_self_balancing_lqr_...md`** — Full mathematical derivation

---

## Tuning Guide

### Simulation Tuning

1. Launch simulation and controller
2. Observe behavior in Gazebo
3. Edit gains in `lqr_balancing_controller.py` (simulation section)
4. Restart the controller (`Ctrl+C` in Terminal 2, re-launch)
5. Analyze CSV logs with `python3 LQR_Calculations/visualize_balancing.py`

### Hardware Tuning

1. **Start with the robot tethered/supported**
2. Edit gains in `lqr_balancing_controller.py` (hardware section)
3. Key parameters to adjust:
   - `THETA_OFFSET` — Compensate for center-of-gravity offset
   - `LEFT_MOTOR_SCALE` / `RIGHT_MOTOR_SCALE` — Compensate for motor imbalance
   - `K_THETA` — Main balancing response (increase for more aggressive correction)
   - `K_THETA_DOT` — Damping (increase to reduce oscillation)
   - `EMERGENCY_TILT_LIMIT` — Safety cutoff angle (degrees)

### Motor Calibration

Run the Kt calibration tool with wheels elevated:

```bash
ros2 run grace_firmware kt_calibration_node
```

This measures the torque constant (`Kt`) for each motor and provides recommended values.

---

## Troubleshooting

### Simulation Issues

| Issue | Solution |
|-------|----------|
| Robot falls immediately | Check that both terminals are running; controller needs pitch data |
| Excessive oscillation | Reduce `K_THETA_DOT` or increase deadband thresholds |
| Robot drifts | Increase `K_P` and `K_P_DOT` to strengthen position holding |
| Gazebo crashes | Ensure correct Gazebo version is installed for your ROS 2 distro |

### Hardware Issues

| Issue | Solution |
|-------|----------|
| No pitch data | Check IMU wiring and Madgwick filter output: `ros2 topic echo /robot_pitch` |
| Motors don't respond | Verify ESP32 port: `ls /dev/grace_esp32` or try `/dev/ttyUSB0` |
| WATCHDOG errors | Sensor data is stale — check serial connection and baud rate |
| Robot leans to one side | Adjust `THETA_OFFSET` in the controller |
| One motor weaker | Adjust `LEFT_MOTOR_SCALE` / `RIGHT_MOTOR_SCALE` |
| Emergency stop triggers | Robot tilted beyond limit — check `EMERGENCY_TILT_LIMIT` |

### Useful ROS 2 Commands

```bash
# Check available topics
ros2 topic list

# Monitor pitch angle
ros2 topic echo /robot_pitch

# Monitor controller output
ros2 topic echo /simple_effort_controller/commands   # simulation
ros2 topic echo /motor_torque_commands                # hardware

# Check joint states
ros2 topic echo /joint_states

# View TF tree
ros2 run tf2_tools view_frames
```

---

## License

This project is licensed under the **Apache License 2.0** — see individual package files for details.

---

## Author

**Muhammad Anss** — [muhammadanss0907@gmail.com](mailto:muhammadanss0907@gmail.com)

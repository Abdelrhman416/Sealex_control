# SEALEX Control System

> **SEALEX** is an autonomous catamaran Unmanned Surface Vessel (USV) engineered for precision marine navigation and automated oil spill remediation. This repository houses the core control algorithms, ROS 2 nodes, simulation environments, and microcontroller integration scripts powering the vessel.

---

## System Architecture & Tech Stack
The control system adopts a distributed architecture combining high-level autonomous navigation with low-level hardware actuation:
* **High-Level Control & Navigation:** ROS 2 Humble (Python / C++) running on a Raspberry Pi 4.
* **Simulation Environment:** Gazebo for hydrodynamic modeling and sensor simulation.
* **Kinematics & Control Design:** MATLAB and Simulink (Line-of-Sight [LOS] guidance, PID heading control, and differential thrust allocation).
* **Low-Level Microcontrollers:** ESP32 handling actuator interfaces, PWM motor signals, and sensor polling.
* **Sensors & Perception:** BNO055 Inertial Measurement Unit (IMU) for orientation and heading feedback.


## Repository Structure

```text
Sealex_control/
├── config/              # ROS 2 parameter files, robot descriptions, and sensor configs
├── launch/              # ROS 2 launch files for bringing up nodes and simulation
├── mat_simulink/        # MATLAB scripts and Simulink models for kinematics & controllers
├── rviz/                # RViz configuration profiles for visualization
├── src/                 # Custom ROS 2 C++ and Python nodes (navigation, control, sensor processing)
├── firmware/            # ESP32 low-level firmware and I2C/Serial communication scripts
└── package.xml          # ROS 2 package manifest

```

## Key Features
* Autonomous Surface Navigation: Implements Line-of-Sight (LOS) guidance algorithms and precise heading controllers to track waypoints smoothly in marine environments.
* Differential Thrust Allocation: Translates surge and yaw commands into individual port and starboard motor speeds for agile maneuvering.
* Oil Spill Remediation Operations: Manages specialized onboard collection and skimming mechanisms optimized for efficient cleanup response.
* Dual Simulation & Hardware Pipeline: Seamlessly transition between Gazebo simulations and physical deployment on the catamaran hull using identical ROS 2 message interfaces.


## Getting Started & Installation

### Prerequisites
Ensure your development environment meets the following requirements:

* **Operating System:** Ubuntu 22.04 LTS (recommended)
* **Middleware:** ROS 2 Humble Hawksbill
* **Math & Modeling:** MATLAB & Simulink (optional, for regenerating controller coefficients)

### Building the Workspace

1. Clone the repository into your ROS 2 workspace `src` directory:
```bash
cd ~/ros2_ws/src
git clone https://github.com/Abdelrhman416/Sealex_control.git
```

2. Install workspace dependencies using `rosdep`:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package using Colcon:
```bash
colcon build --packages-select sealex_control
```

4. Source your workspace:
```bash
source install/setup.bash
```

## Usage
### Running the Simulation
To launch the USV in the Gazebo environment along with RViz visualization:
```bash
ros2 launch sealex_control simulation.launch.py
```

### Deploying on Hardware
To start the core control nodes and establish serial communication with the ESP32 and BNO055 IMU on the physical vessel:
```bash
ros2 launch sealex_control hardware_bringup.launch.py
```

## License
This project is developed as part of advanced mechatronics and autonomous marine robotics research. Feel free to reference or build upon the architecture.

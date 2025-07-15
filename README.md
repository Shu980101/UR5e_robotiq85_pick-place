# 🤖 UR5e Robotiq85 Pick-and-Place in ROS 2 & Gazebo

<hr>

<div align="center">
  <b>UR5e Pick-and-Place Simulation using MoveIt 2, Robotiq 2F-85, and Link Attacher</b><br>
  A ROS 2-based robotic manipulation pipeline for simulating grasping and placement tasks in Gazebo using ArUco-calibrated objects and service-based link attachment.
</div>

<br>

<div align="center">

![ROS 2](https://img.shields.io/badge/ROS_2-Humble-blue)
![URobot](https://img.shields.io/badge/Robot-UR5e-green)
![Gripper](https://img.shields.io/badge/Gripper-Robotiq%202F85-orange)
![Gazebo](https://img.shields.io/badge/Simulator-Gazebo_Classic-yellow)
![License](https://img.shields.io/badge/License-MIT-lightgrey)

</div>

---

## 🎥 Demo Video

A ROS 2-based robotic manipulation pipeline for simulating grasping and placement tasks in Gazebo using ArUco-calibrated objects and service-based link attachment.

> 📺 **Demo Video:** [https://youtu.be/DGz_4a_z19A](https://youtu.be/DGz_4a_z19A)

---

## 🧠 Project Overview

This project demonstrates a complete ROS 2-based pick-and-place pipeline using the **UR5e** robotic arm and **Robotiq 2F-85 gripper** in **Gazebo Classic**. It integrates **MoveIt 2** for motion planning and uses the **IFRA Link Attacher plugin** to simulate object grasping and releasing without needing Gazebo plugins inside object URDFs.

---

## 🚀 Quick Start

### ✅ Requirements

- **OS**: Ubuntu 22.04
- **ROS 2**: Humble
- **Simulation**: Gazebo Classic
- **Robot**: UR5e with Robotiq 2F-85
- **Dependencies**:
  - `gazebo_ros_link_attacher`
  - `Universal_Robots_ROS2_Driver`
  - `Universal_Robots_ROS2_Description`
  - `Universal_Robots_ROS2_Gazebo_Simulation`
  - `ros2_control` and `MoveIt 2`

---

## 📦 Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Correct: clone into src/ (not src/src)
git clone https://github.com/Shu980101/UR5e_robotiq85_pick-place.git

# Go back to workspace root
cd ~/ros2_ws

# Install dependencies
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build
source install/setup.bash
```

## 📦 Usage

### View robot only
```bash
ros2 launch ur5e_golf_pick_place view_ur5e.launch.py 
```

### View robot with Moveit2 and gazebo
```bash
ros2 launch ur5e_golf_pick_place ur5_moveit.launch.py
```

### View robot with Robotiq85 gripper
```bash
ros2 launch ur5e_golf_pick_place view_ur5e_robotiq.launch.py
```

### View robot with Robotiq85 gripper in Moveit2 and Gazebo
```bash
ros2 launch ur5e_golf_pick_place ur5e_robotiq_moveit.launch.py
```

Once you've launched the `ur5e_robotiq_moveit.launch.py` file, navigate to `/ros2_ws/src/ur5e_golf_pick_place/ur5e_golf_pick_place/Pick_and_place.py` and run it. The script will automatically execute a basic pick-and-place task.

## 📌 Source Credits

This project integrates and builds upon the following open-source ROS 2 packages:

- [ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper)  
  Robotiq 2F-85 gripper description, controller config, and MoveIt 2 integration.

- [IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher)  
  Simulates grasping and detachment using `/ATTACHLINK` and `/DETACHLINK` services in Gazebo.

- [Universal_Robots_ROS2_Description (Humble)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble)  
  URDF/Xacro robot description packages for UR5e, UR10e, etc.

- [Universal_Robots_ROS2_Gazebo_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation)  
  Gazebo Classic support with ROS 2 integration for UR robots.

- [Universal_Robots_ROS2_Driver (Humble)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)  
  The official ROS 2 driver to interface with Universal Robots via `ros2_control` and MoveIt 2.


## ⚠ Known Issue: Robotiq Gripper Simulation

The Robotiq 2F-85 gripper is **not fully functional in Gazebo Classic** due to limited support for realistic finger joint actuation and contact simulation.

As a workaround, this project uses the [IFRA_LinkAttacher](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher) plugin to **simulate grasp and release actions** by attaching and detaching the object from the gripper link directly via service calls.

This approach provides a stable simulation for pick-and-place logic, but does not reflect actual finger movement.  
🔧 **An improved gripper control solution will be implemented in a future update.**


## 👤 Author

**Shu Xiao**  
Master in Robotics and Advanced Construction  
[IAAC Barcelona](https://www.iaac.net)

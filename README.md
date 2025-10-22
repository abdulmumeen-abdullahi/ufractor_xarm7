# Ufactory Xarm7 – Aurora Robotics Workshop Task 4

This project focuses on assembling, describing, and visualizing the **Ufactory Xarm7** robotic arm using **ROS 2** and the **Unified Robot Description Format (URDF)**.  
The assembly strictly follows the [Ufactory Documentation](https://docs.ufactory.cc/user_manual/ufactoryStudio/11.technical_specifications.html).

<img width="1366" height="768" alt="Ufactory Xarm7" src="https://github.com/user-attachments/assets/2a6973c7-57f5-4c7b-8bef-4acc0ae5a83b" />


## Overview

The project defines the full URDF structure for the Xarm7, including links, joints, and visual meshes with assigned materials.  
Each link references its mesh in the `ufactory_xarm7_description/meshes` directory, and the arm is fully visualizable in **Rviz**.  
Simulation and advanced control (MoveIt, Gazebo/Ignition) can be added in later phases.


## Key Specifications

All revolute joints have:

- **Effort:** maximum allowable torque or force of **3 Nm**  
- **Velocity:** maximum rotational speed of **3.14 rad/s (180°/s)**  

The URDF also applies a **white material** for all links for consistent visualization.


## Joint Configuration

| Joint Name           | Type      | Lower Limit (rad) | Upper Limit (rad) | Lower Limit (°) | Upper Limit (°) |
|----------------------|-----------|------------------|------------------|----------------|----------------|
| base_link_to_link1   | Revolute  | -6.283           | 6.283            | -360°          | 360°           |
| link1_to_link2       | Revolute  | -2.059           | 2.094            | -118°          | 120°           |
| link2_to_link3       | Revolute  | -6.283           | 6.283            | -360°          | 360°           |
| link3_to_link4       | Revolute  | -0.192           | 3.927            | -11°           | 225°           |
| link4_to_link5       | Revolute  | -6.283           | 6.283            | -360°          | 360°           |
| link5_to_link6       | Revolute  | -1.692           | 3.142            | -97°           | 180°           |
| link6_to_link7       | Revolute  | -6.283           | 6.283            | -360°          | 360°           |
| link7_to_gripper     | Fixed     | –                | –                | –              | –              |

## Description

Each link includes visual meshes and assigned **white material** for clear visualization in Rviz.  
Joints define spatial relationships (`xyz` and `rpy` origins), parent-child hierarchy, rotation axes, and joint limits.

### URDF Highlights

- **Material:** All links use `<material name="white"/>`  
- **Base link:** `base_link` with initial fixed pose  
- **Gripper:** Fixed joint at the end-effector for visualization purposes  
- **Revolute joints:** Follow Xarm7 velocity limits  

## Next Steps
 
- **Dynamic Simulation:** Integration with Gazebo or Ignition  
- **Control & Planning:** MoveIt setup for motion planning  
- **Sensor & Gripper Modeling:** Extend URDF to include sensors, gripper interactions 

## Quick Launch (ROS 2 + Rviz)

Follow these steps to visualize the Xarm7 in Rviz:

### 1. Install ROS 2 and Required Packages

Make sure you have a ROS 2 distribution installed (Humble recommended).  
Then install the necessary packages:

```bash
sudo apt update
sudo apt install ros-humble-urdf-tutorial \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-robot-state-publisher \
                 ros-humble-rviz2
Replace humble with your ROS 2 distro if needed.
```

2. **Create the workspace and Clone the repository**  

```bash
mkdir ufractor_xarm7_ws
cd ufractor_xarm7_ws
git clone https://github.com/abdulmumeen-abdullahi/ufractor_xarm7.git

```

3. **Build the ROS 2 package**

```bash
colcon build --symlink-install
source install/setup.bash
```

4. **Launch Rviz with the Xarm7 URDF**

```bash
ros2 launch urdf_tutorial display.launch.py model:=$PWD/ufactory_xarm7_description/urdf/ufactory_xarm7.urdf
```
- $PWD should point to the root of your cloned repository.
- The ufactory_xarm7.urdf file must exist under ufactory_xarm7_description/urdf/.

5. **Visualize and Interact**

- Rviz will open with the Xarm7 robot loaded.
- Use the Joint State Publisher GUI to move revolute joints interactively.
- The robot’s links are displayed with white material for clarity.


## Next Steps

- Dynamic Simulation: Integration with Gazebo or Ignition
- Control & Planning: MoveIt setup for motion planning
- Sensor & Gripper Modeling: Extend URDF to include sensors and gripper interactions

*Developed as part of the [Aurora Robotics](https://ng.linkedin.com/company/aurora-robotics-in) Core Robotics Workshop – Task 4.*

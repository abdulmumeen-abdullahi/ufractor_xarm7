# Ufactory Xarm7 – Aurora Robotics Workshop Task 4

This project focuses on assembling, describing, and visualizing the **Ufactory Xarm7** robotic arm using **ROS 2** and the **Unified Robot Description Format (URDF)**.  
The assembly strictly follows the [Ufactory Documentation](https://docs.ufactory.cc/user_manual/ufactoryStudio/11.technical_specifications.html).


## Overview

The project defines the full URDF structure for the Xarm7, including links, joints, and visual meshes with assigned materials.  
Each link references its mesh in the `ufactory_xarm7_description/meshes` directory, and the arm is fully visualizable in **Rviz**.  
Simulation and advanced control (MoveIt, Gazebo/Ignition) can be added in later phases.

## Key Specifications

All revolute joints have:

- **Effort:** maximum allowable torque or force of **3 Nm** for a start  
- **Velocity:** maximum rotational speed of **3.14 rad/s (180°/s)**, as specified in the Ufactory documentation  

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

## Next Steps / Applications

- **Visualization:** Launch in Rviz for kinematic inspection  
- **Dynamic Simulation:** Integration with Gazebo or Ignition  
- **Control & Planning:** MoveIt setup for motion planning  
- **Sensor & Gripper Modeling:** Extend URDF to include sensors, gripper interactions 

## Quick Launch (ROS 2 + Rviz)

Follow these steps to visualize the Xarm7 in Rviz:

1. **Clone the repository**  

```bash
git clone https://github.com/abdulmumeen-abdullahi/ufractor_xarm7.git
cd ufractor_xarm7
```

2. **Build the ROS 2 package**

```bash
colcon build --symlink-install
source install/setup.bash
```

2. **Launch Rviz with the Xarm7 URDF**

```bash
ros2 launch urdf_tutorial display.launch.py model:=$PWD/ufactory_xarm7_description/urdf/ufactory_xarm7.urdf
```

> This will open Rviz with the robot model loaded. Use the Joint State Publisher to move revolute joints interactively.

## Next Steps

Dynamic Simulation: Integration with Gazebo or Ignition

Control & Planning: MoveIt setup for motion planning

Sensor & Gripper Modeling: Extend URDF to include sensors and gripper interactions

*Developed as part of the [Aurora Robotics](https://ng.linkedin.com/company/aurora-robotics-in) Core Robotics Workshop – Task 4.*
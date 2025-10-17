# Ufactory Xarm7 – Aurora Robotics Workshop Task 4

This project focuses on assembling, describing, and visualizing the Ufactory Xarm7 robotic arm using ROS 2 and the Unified Robot Description Format (URDF).  
The assembly strictly follows the [Ufactory Documentation](https://docs.ufactory.cc/user_manual/ufactoryStudio/11.technical_specifications.html).

## Overview

The project defines the full URDF structure for the Xarm7, including links, joints, and visual meshes.  
Visualization is done using Rviz, while simulation will be implemented in later phases.

## Key Specifications

All revolute joints have:

- **Effort:** maximum allowable torque or force of 3 Nm
- **Velocity:** maximum rotational speed of 3.14 rad/s (180°/s), as specified in the Ufactory documentation


## Joint Configuration

| Joint Name           | Type      | Lower Limit (rad) | Upper Limit (rad) | Lower Limit (°) | Upper Limit (°) |
|----------------------|-----------|-------------------|-------------------|------------------|------------------|
| base_link_to_link1   | Fixed     | –                 | –                 | –                | –                |
| link1_to_link2       | Revolute  | -6.283            | 6.283             | -360°            | 360°             |
| link2_to_link3       | Revolute  | -2.059            | 2.094             | -118°            | 120°             |
| link3_to_link4       | Revolute  | -6.283            | 6.283             | -360°            | 360°             |
| link4_to_link5       | Revolute  | -0.192            | 3.927             | -11°             | 225°             |
| link5_to_link6       | Revolute  | -6.283            | 6.283             | -360°            | 360°             |
| link6_to_link7       | Revolute  | -1.692            | 3.142             | -97°             | 180°             |
| link7_to_gripper     | Revolute  | -6.283            | 6.283             | -360°            | 360°             |

---

## Description

Each link in the robot’s kinematic chain references its corresponding mesh file located in the `ufactory_xarm7_description/meshes` directory.  
The joints define their spatial relationships (`xyz` and `rpy` origins), parent-child hierarchy, rotation axes, and joint limits.

This URDF model forms the foundation for further work, including:

- **Dynamic simulation** using Gazebo or Ignition  
- **Control integration** with MoveIt  
- **Sensor and gripper modeling**

*Developed as part of the [Aurora Robotics](https://ng.linkedin.com/company/aurora-robotics-in) Core Robotics Workshop – Task 4.*

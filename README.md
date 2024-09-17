# 4-Wheeled Holonomic System

## Project Overview
The **4-Wheeled Holonomic System** is a robotic platform that utilizes a **4-wheel holonomic drive system** to achieve omnidirectional movement. Unlike traditional drive systems, holonomic wheels (also known as omni-wheels) enable the rover to move in any direction (forward, backward, side-to-side, and diagonally) without the need for turning its chassis. This provides unmatched maneuverability, making the platform suitable for applications in robotics research, automation, and environments requiring precise mobility.

## Features
- **Omni-directional Movement**: Can move in any direction without the need to rotate the base.
- **4-Wheel Holonomic Drive**: Utilizes four omni-wheels for smooth and flexible movement.
- **Highly Maneuverable**: Capable of complex movements including strafing and rotating simultaneously.
- **Compact Design**: Optimized for both indoor and outdoor environments, capable of navigating tight spaces.
- **Scalable**: Design can be adapted for different sizes of platforms depending on the application.

### Movement Algorithm
The algorithm for the **4-Wheeled Holonomic System** leverages vector math to calculate the required wheel velocities for the desired movement. It takes into account:
- Translation along the X and Y axes
- Rotation about the Z-axis
- The alogorithm utilizes Fuzzy logic to maintain the smooth manoeurability of the system.  

# Control Task: Modeling and Controlling a Differential Drive Vehicle with PID Controller

## Objective
Design and implement a PID (Proportional-Integral-Derivative) controller to control a differential drive vehicle to follow a predefined circular trajectory with stable and accurate navigation.

## Task Description
You are tasked with modeling the kinematics of a differential drive vehicle and designing a PID controller to regulate its motion along a predefined circular trajectory. The goal is to ensure the vehicle maintains the desired speed and orientation while following the circular path with minimal deviation.

## Requirements
1. **Vehicle Model**: Develop a kinematic model for a differential drive vehicle, accounting for wheel velocities and vehicle pose (position and orientation).
2. **PID Controller**: Design a PID controller to regulate the vehicle's wheel velocities to follow the specified circular trajectory.
3. **Circular Trajectory**: The vehicle must follow a circular trajectory with a radius of 5 meters, centered at the origin (0, 0), starting at position (5, 0) with an initial orientation of 0 degrees (facing along the positive y-axis). The trajectory is defined by the following waypoints (x, y coordinates in meters), sampled at 10-degree intervals over one full circle (360 degrees):
   - (5.000, 0.000), (4.966, 0.868), (4.866, 1.710), (4.698, 2.500), (4.466, 3.214), (4.173, 3.830), (3.830, 4.330), (3.441, 4.698), (2.989, 4.940), (2.500, 5.000), (1.985, 4.940), (1.423, 4.698), (0.868, 4.330), (0.294, 3.830), (-0.294, 3.214), (-0.868, 2.500), (-1.423, 1.710), (-1.985, 0.868), (-2.500, 0.000), (-1.985, -0.868), (-1.423, -1.710), (-0.868, -2.500), (-0.294, -3.214), (0.294, -3.830), (0.868, -4.330), (1.423, -4.698), (1.985, -4.940), (2.500, -5.000), (2.989, -4.940), (3.441, -4.698), (3.830, -4.330), (4.173, -3.830), (4.466, -3.214), (4.698, -2.500), (4.866, -1.710), (4.966, -0.868), (5.000, 0.000).

   [(5.0, 0.0), (4.924, 0.868), (4.698, 1.71), (4.33, 2.5), (3.83, 3.214), (3.214, 3.83), (2.5, 4.33), (1.71, 4.698), (0.868, 4.924), (0.0, 5.0), (-0.868, 4.924), (-1.71, 4.698), (-2.5, 4.33), (-3.214, 3.83), (-3.83, 3.214), (-4.33, 2.5), (-4.698, 1.71), (-4.924, 0.868), (-5.0, 0.0), (-4.924, -0.868), (-4.698, -1.71), (-4.33, -2.5), (-3.83, -3.214), (-3.214, -3.83), (-2.5, -4.33), (-1.71, -4.698), (-0.868, -4.924), (-0.0, -5.0), (0.868, -4.924), (1.71, -4.698), (2.5, -4.33), (3.214, -3.83), (3.83, -3.214), (4.33, -2.5), (4.698, -1.71), (4.924, -0.868), (5.0, -0.0)]

4. **Performance Metrics**: Evaluate the controller's performance using metrics such as path deviation, settling time, and overshoot.

## Deliverables
- A report (PDF) describing the kinematic model, PID controller design (including tuning methodology), and simulation setup.
- Visualizations (plots or animations) showing the vehicle's motion along the circular trajectory and the controller's performance metrics.
- A discussion of challenges faced and how they were addressed.

## Notes
- You may use any programming language or simulation environment (e.g., Python with Matplotlib, MATLAB, or ROS).
- Clearly document any assumptions made about the vehicle's parameters (e.g., wheelbase, wheel radius).
- The waypoints are provided to guide the trajectory; you may interpolate between them for smoother control.
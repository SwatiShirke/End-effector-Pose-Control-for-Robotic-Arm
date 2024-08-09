# Robot Fanuc LR Mate 200iC Pose Control System

## Overview

This project implements end-effector pose control for a 6-DOF (Degrees of Freedom) Fanuc LR Mate 200iC robotic manipulator using the Recursive Newton-Euler (RNE) algorithm. The project features a MATLAB-based graphical user interface (GUI) for user interaction, allowing for pose and mass load inputs, and providing real-time simulation and visualization of the robot's performance.

## Key Features

1. **End-Effector Pose Control:**
   - Utilized Recursive Newton-Euler algorithm to control the end-effector's pose.
   - Enabled precise movement and positioning of the Fanuc LR Mate 200iC robot arm.

2. **MATLAB GUI Interface:**
   - Designed and developed a GUI in MATLAB to facilitate user inputs for pose and mass load.
   - The interface provides real-time simulation of the robot and displays:
     - Joint torque
     - Pose
     - Velocities
   - Interactive visualizations help in understanding the robot's behavior under different conditions.

3. **Inverse Kinematics Calculations:**
   - Implemented inverse kinematics to determine the required joint torques and velocities for achieving the desired end-effector pose.
   - The robot's motion was controlled to execute precise tasks as per the calculated parameters.

## Implementation

- **Recursive Newton-Euler Algorithm:** Used for efficient computation of joint torques and velocities based on the desired end-effector pose.
- **MATLAB Robotics Toolbox:** Leveraged Peter Corke’s MATLAB Robotics Toolbox for advanced robotics functions and simulation.
- **GUI Development:** Created in MATLAB to provide a user-friendly interface for pose input and to visualize the robot’s simulation.

## Results

The project successfully demonstrates the control of the Fanuc LR Mate 200iC robot's end-effector pose with the following results:
- Accurate simulation of the robot’s movement in response to user inputs.
- Real-time display of joint torques, end-effector pose, and velocities.
- Effective use of inverse kinematics to achieve desired robot motions.

### Example Results

![Simulation Results](path/to/your/image.png)

*Note: Replace `path/to/your/image.png` with the actual path to your result image.*

## Future Work

- **Enhanced GUI Features:** Add more functionalities such as path planning and collision detection.
- **Real-Time Implementation:** Integrate the system with a physical robot for real-time pose control and feedback.
- **Optimization:** Explore optimization techniques for more efficient calculations and better performance.

## References

- Peter Corke’s MATLAB Robotics Toolbox: [Link to Toolbox](https://www.mathworks.com/matlabcentral/fileexchange/11306-robotics-toolbox-for-matlab)

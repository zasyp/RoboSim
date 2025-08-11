## Waffer handling robot simulation written in Python framework exudyn
https://github.com/user-attachments/assets/f5254741-99d3-4e21-9ba8-0a6bfa40289e
This script simulates a robotic manipulator with a kinematic tree using the EXUDYN library. It defines a 4-link robot with a base, cylindrical link, and three rotational links, controlled via a predefined trajectory.

Robot Definition: Configures a robot with a base and four links (1 prismatic, 3 revolute joints).
Trajectory Control: Implements a trajectory with constant acceleration profiles for joint movements.
Visualization: Includes graphical representations for the base and links (box, cylinder, custom bodies).
Sensors: Tracks joint displacement and rotation.
Simulation: Runs a dynamic simulation with a trapezoidal index-2 solver.

The script uses a kinematic tree (ObjectKinematicTree) for robot dynamics.
Trajectory points are defined in q1 to q5 with constant acceleration profiles.

## Usage
Ensure all required constants are defined in helpful.constants.
Run the script to simulate the robot's motion over 6 seconds with a 0.25ms step size.
Visualize results using the EXUDYN SolutionViewer.

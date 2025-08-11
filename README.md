## Wafer Handling Robot Simulation in Python (Exudyn)
![animation](https://github.com/user-attachments/assets/1e855364-8909-4ac7-be5f-1b404086c368)
---
This project demonstrates the simulation of a wafer handling robot manipulator using the [Exudyn](https://exudyn.readthedocs.io/) multibody dynamics library.
The robot is modeled as a kinematic tree (`ObjectKinematicTree`) with a prismatic vertical axis and three rotational joints for planar motion.
* **Robot Definition**
Robot defined by robotics.Robot class. Base tool and four links with masses, inertias and simple STL format 3D models that you can find in graphics directory. All the physical constants can be modified in helpful.constants.py file.
* **Trajectory Control**
 At this point trajectory is realised by exudyn built in functions. You can define motion pattern by setting joint despositions (forward kinematics).
---
### **Installation**
1. Install Python 3.9+ (recommended).
2. Install required packages:
   ```bash
   pip install exudyn numpy matplotlib
   ```
3. Clone or download this repository.
---
### **Usage**
There are two directories MBS realisation and robotics realisation. MBS defines robot in simple exudyn multybody objects, robotics use exudyn.robotics module. Robotics are prefferable.
Run the main simulation script (roboticsForward.py or knFWhypotesys.py):
   ```bash
   python knFWhypotesys.py
   ```
### **Output**
* Animated simulation in the Exudyn window.
* Logged joint motion data for further processing.
* Graphical visualizations of position, velocity, and control error.
* Torque and forces plots.

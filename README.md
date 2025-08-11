## Wafer Handling Robot Simulation in Python (Exudyn)

![animation](https://github.com/user-attachments/assets/1e855364-8909-4ac7-be5f-1b404086c368)

This project demonstrates the simulation of a wafer handling robot manipulator using the [Exudyn](https://exudyn.readthedocs.io/) multibody dynamics library.
The robot is modeled as a kinematic tree (`ObjectKinematicTree`) with a prismatic vertical axis and three rotational joints for planar motion.

---

### **Features**

* **Robot Definition**

  * Base + 4 links:

    * **1 prismatic joint** (vertical Z-axis)
    * **3 revolute joints** (rotation around Z-axis)
  * Physical parameters: link masses, inertia tensors, COM positions.
  * Graphical models for base and links (box, cylinder, and custom STL shapes).

* **Trajectory Control**

  * Motion defined using constant acceleration profiles (`ProfileConstantAcceleration`).
  * Predefined joint positions `q1` to `q5`.
  * PD control for joint movement.

* **Sensors**

  * Joint position, velocity, and acceleration tracking via `SensorKinematicTree`.
  * Data logging for post-simulation analysis.

* **Simulation**

  * Time step: `h = 0.25 × 10⁻³ s`
  * Duration: `6 s`
  * Solver: **TrapezoidalIndex2** (implicit integration).
  * Control implemented in `PreStepUserFunction`.

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

1. Define constants and geometry in `helpful.constants`.
2. Run the main simulation script:

   ```bash
   python wafer_robot.py
   ```
3. Use **Exudyn SolutionViewer** to visualize simulation results:

   ```python
   from exudyn.interactive import SolutionViewer
   SolutionViewer(mbs, solutionFile)
   ```

---

### **Output**

* Animated simulation in the Exudyn window.
* Logged joint motion data for further processing.
* Graphical visualizations of position, velocity, and control error.

---

### **Applications**

* Wafer handling robots in semiconductor manufacturing.
* Educational purposes for learning multibody simulation in Python.
* Testing control algorithms for multi-DOF robotic systems.

---

### **References**

* [Exudyn Documentation](https://exudyn.readthedocs.io/)

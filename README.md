## Wafer Handling Robot Simulation in Python (Exudyn)
![animation](https://github.com/user-attachments/assets/1e855364-8909-4ac7-be5f-1b404086c368)
---
This project is a robotics simulation using Exudyn for multibody dynamics. It includes trajectory generation, dynamic simulation, and plotting of sensor signals and joint torques.
## Project Structure
- `roboticsForward.py` – Main forward-dynamics simulation script
- `src/trajectory_opt.py` – Trajectory generation and time-parameterization
- `plot_sensors.py` – Plotting from files in `sensor_outputs/` (no simulation run)
- `run_simulation.py` – One-click runner: simulation + plots
- `constants.py` – Robot parameters and visualization assets
- `tests.py` – Helpers for q(t), q̇(t), q̈(t)
- `sensor_outputs/` – Simulation outputs (CSV-like text files)
- `plots/` – Generated figures
### **Installation**
1. Install Python 3.9+ (recommended).
2. Install required packages:
   ```bash
   pip install requirements
   ```
3. Clone or download this repository.
---
## Usage
- Run the full pipeline (simulation + plots):
```bash
python run_simulation.py
```
- Run only simulation (writes files to `sensor_outputs/`):
```bash
python roboticsForward.py
```
- Generate plots only (reads existing files in `sensor_outputs/`):
```bash
python plot_sensors.py
```
## What gets plotted
`plot_sensors.py` generates the following figures in `plots/`:
- `vertical_velocity.png` – Vertical velocity
- `omega1.png`, `omega2.png`, `omega3.png` – Joint angular velocities
- `vertical_acceleration.png` – Vertical acceleration
- `epsilon1.png`, `epsilon2.png`, `epsilon3.png` – Joint angular accelerations
- `tau_ff.png` – Feed-forward torques over time (see below)
## Joint torques
- Torques are computed in `roboticsForward.py` in `PreStepUF` (feed-forward term) and written by Exudyn into `sensor_outputs/Torques.txt` (or compatible file names).
- `plot_sensors.py` contains a robust parser for torque files. It:
  - accepts comma-, space-, or bracketed formats
  - extracts all numbers via regex per line
  - assumes the first column is time, the remaining columns are torques for the joints
- The plot is saved as `plots/tau_ff.png`. If no valid torque file is found or parsing fails, the script will skip the torque plot with a warning.
## Notes
- The simulation window is handled programmatically; deprecated Exudyn calls were replaced with the recommended API.
- If plots look empty, ensure that `sensor_outputs/` contains fresh data by re-running `roboticsForward.py` first.
- The repository uses binary STL assets in `graphics/` for visualization. 
---


### **Output**
* Animated simulation in the Exudyn window.
* Logged joint motion data for further processing.
* Graphical visualizations of position, velocity, and control error.
* Torque and forces plots.

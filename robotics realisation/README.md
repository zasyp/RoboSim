# RoboSim Robotics Simulation

This project is a robotics simulation using Exudyn for multibody dynamics. It includes trajectory generation, dynamic simulation, and plotting of sensor signals and joint torques.

## Project Structure

- `roboticsForward.py` – Main forward-dynamics simulation script
- `src/trajectory_opt.py` – Trajectory generation and time-parameterization
- `plot_sensors.py` – Plotting from files in `sensor_outputs/` (no simulation run)
- `run_simulation.py` – One-click runner: simulation + plots
- `constants.py` – Robot parameters and visualization assets
- `tests.py` – Helpers for q(t), q̇(t), q̈(t) and ideal trajectory wrapper
- `sensor_outputs/` – Simulation outputs (CSV-like text files)
- `plots/` – Generated figures

## Requirements

Install dependencies in the repository root:

```bash
pip install -r ../requirements.txt
```

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
- `vertical_velocity.png` – Vertical velocity vs ideal
- `omega1.png`, `omega2.png`, `omega3.png` – Joint angular velocities vs ideal
- `vertical_acceleration.png` – Vertical acceleration vs ideal
- `epsilon1.png`, `epsilon2.png`, `epsilon3.png` – Joint angular accelerations vs ideal
- `err_*.png` – Error plots for the above signals
- `all_torques.png` – Joint torques over time (see below)

## Joint torques

- Torques are computed in `roboticsForward.py` in `PreStepUF` (feed-forward term) and written by Exudyn into `sensor_outputs/Torques.txt` (or compatible file names).
- `plot_sensors.py` contains a robust parser for torque files. It:
  - accepts comma-, space-, or bracketed formats
  - extracts all numbers via regex per line
  - assumes the first column is time, the remaining columns are torques for the joints
- The plot is saved as `plots/all_torques.png`. If no valid torque file is found or parsing fails, the script will skip the torque plot with a warning.

## Notes

- The simulation window is handled programmatically; deprecated Exudyn calls were replaced with the recommended API.
- If plots look empty, ensure that `sensor_outputs/` contains fresh data by re-running `roboticsForward.py` first.
- The repository uses binary STL assets in `graphics/` for visualization. 
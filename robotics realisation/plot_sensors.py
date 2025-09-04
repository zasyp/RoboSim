import os
import numpy as np
import matplotlib.pyplot as plt
from tests import *
import re
from src.trajectory_opt import forward_kin, Params
from constants import l1, l2, l3

# Resolve repo root from this file location
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(_THIS_DIR, os.pardir))
SENSOR_DIR = os.path.join(ROOT_DIR, 'sensor_outputs')
PLOTS_DIR = os.path.join(ROOT_DIR, 'plots')


def get_sensor_data_exudyn():
    """Try to read sensor data directly from Exudyn memory (preferred). Returns dict or None."""
    try:
        # Importing roboticsForward after a simulation run lets us access stored sensor data without files
        import roboticsForward as rf
        mbs = rf.mbs
        data = {
            "verticalDisp": mbs.GetSensorStoredData(rf.verticalDispSens),
            "verticalVel":  mbs.GetSensorStoredData(rf.verticalVelSens),
            "theta1":       mbs.GetSensorStoredData(rf.theta1Sensor),
            "theta2":       mbs.GetSensorStoredData(rf.theta2Sensor),
            "theta3":       mbs.GetSensorStoredData(rf.theta3Sensor),
            "omega1":       mbs.GetSensorStoredData(rf.omega1Sensor),
            "omega2":       mbs.GetSensorStoredData(rf.omega2Sensor),
            "omega3":       mbs.GetSensorStoredData(rf.omega3Sensor),
            "torques":      np.array([]),  # Initialize empty, will be filled from files if needed
        }
        # Basic validation: ensure arrays are non-empty
        if any(isinstance(v, np.ndarray) and v.size > 0 for v in data.values()):
            return data
    except Exception:
        pass
    return None


def get_sensor_data_from_files():
    """Read sensor data from files instead of running simulation"""
    output_dir = SENSOR_DIR
    
    # Read data from files
    data = {}
    
    # Read vertical displacement
    try:
        vertical_disp_data = np.loadtxt(os.path.join(output_dir, "verticalDisp.txt"), 
                                       comments='#', delimiter=',')
        data["verticalDisp"] = vertical_disp_data
    except:
        print("Warning: Could not read verticalDisp.txt")
        data["verticalDisp"] = np.array([])
    
    # Read theta1
    try:
        theta1_data = np.loadtxt(os.path.join(output_dir, "theta1_deg.txt"), 
                                 comments='#', delimiter=',')
        data["theta1"] = theta1_data
    except:
        print("Warning: Could not read theta1_deg.txt")
        data["theta1"] = np.array([])
    
    # Read theta2
    try:
        theta2_data = np.loadtxt(os.path.join(output_dir, "theta2_deg.txt"), 
                                 comments='#', delimiter=',')
        data["theta2"] = theta2_data
    except:
        print("Warning: Could not read theta2_deg.txt")
        data["theta2"] = np.array([])
    
    # Read theta3
    try:
        theta3_data = np.loadtxt(os.path.join(output_dir, "theta3_deg.txt"), 
                                 comments='#', delimiter=',')
        data["theta3"] = theta3_data
    except:
        print("Warning: Could not read theta3_deg.txt")
        data["theta3"] = np.array([])
    
    # Read vertical velocity
    try:
        vertical_vel_data = np.loadtxt(os.path.join(output_dir, "verticalVel.txt"), 
                                       comments='#', delimiter=',')
        data["verticalVel"] = vertical_vel_data
    except:
        print("Warning: Could not read verticalVel.txt")
        data["verticalVel"] = np.array([])
    
    # Read omega1
    try:
        omega1_data = np.loadtxt(os.path.join(output_dir, "omega1_deg.txt"), 
                                 comments='#', delimiter=',')
        data["omega1"] = omega1_data
    except:
        print("Warning: Could not read omega1_deg.txt")
        data["omega1"] = np.array([])
    
    # Read omega2
    try:
        omega2_data = np.loadtxt(os.path.join(output_dir, "omega2_deg.txt"), 
                                 comments='#', delimiter=',')
        data["omega2"] = omega2_data
    except:
        print("Warning: Could not read omega2_deg.txt")
        data["omega2"] = np.array([])
    
    # Read omega3
    try:
        omega3_data = np.loadtxt(os.path.join(output_dir, "omega3_deg.txt"), 
                                 comments='#', delimiter=',')
        data["omega3"] = omega3_data
    except:
        print("Warning: Could not read omega3_deg.txt")
        data["omega3"] = np.array([])

    # Read torques
    data["torques"] = load_torques_from_files()
    
    return data

def load_torques_from_files():
    """Load torque data from files"""
    output_dir = SENSOR_DIR
    
    # Read torques - try multiple common file names
    torque_files = ["tau_ff.txt", "Torques.txt", "torques.txt", "torque_data.txt"]
    torques = None
    for filename in torque_files:
        try:
            filepath = os.path.join(output_dir, filename)
            if os.path.exists(filepath):
                torques = np.loadtxt(filepath, delimiter=',', comments='#')
                print(f"Successfully loaded torques from {filename}")
                break
        except Exception as e:
            print(f"Warning: Could not read {filename}: {e}")
            continue
    
    return torques if torques is not None else np.array([])

def save_plot(x, y_list, labels, title, ylabel, filename, linestyles=None, outdir=PLOTS_DIR):
    os.makedirs(outdir, exist_ok=True)
    plt.figure(figsize=(8, 6))
    for i, (y, lbl) in enumerate(zip(y_list, labels)):
        # Skip plotting if data is empty or has wrong dimensions
        if len(y) == 0 or len(y) != len(x):
            print(f"Warning: Skipping {lbl} - data has wrong dimensions or is empty")
            continue
        style = linestyles[i] if linestyles and i < len(linestyles) else "-"
        plt.plot(x, y, style, label=lbl)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(ylabel)
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, filename), dpi=300)
    plt.close()

def save_xy_plot(xy_series, labels, title, filename, outdir=PLOTS_DIR):
    os.makedirs(outdir, exist_ok=True)
    plt.figure(figsize=(7, 7))
    for (xs, ys), lbl in zip(xy_series, labels):
        plt.plot(xs, ys, label=lbl)
    plt.axis('equal')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title(title)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(outdir, filename), dpi=300)
    plt.close()

def plot_torques(torques, times=None, outdir=PLOTS_DIR):
    """Easy torque plotting function"""
    if not isinstance(torques, np.ndarray) or torques.size == 0:
        print("Warning: No torque data available")
        return
    
    # Handle different data formats
    if torques.ndim == 1:
        torques = torques.reshape(1, -1)
    
    # Determine time and torque data
    if torques.shape[1] > 1:
        t_tau = torques[:, 0]
        torque_data = [torques[:, i] for i in range(1, torques.shape[1])]
    else:
        if times is None:
            print("Warning: No time data available for torques")
            return
        t_tau = times
        torque_data = [torques.flatten()]
    
    # Create labels
    labels = [f"τ{i}" for i in range(len(torque_data))]
    
    # Save plot
    save_plot(t_tau, torque_data, labels, "Joint Torques", "Torque (N·m)", "tau_ff.png", outdir=outdir)
    print(f"Torque plot saved with {len(torque_data)} joint(s)")


def main():
    # Prefer Exudyn in-memory data; fallback to files
    data = get_sensor_data_exudyn()
    if data is None:
        data = get_sensor_data_from_files()
    else:
        # If we got data from Exudyn memory, still try to load torques from files
        data["torques"] = load_torques_from_files()
    
    # Check if we have any data
    if not any(isinstance(v, np.ndarray) and v.size > 0 for v in data.values()):
        print("No sensor data found. Please run roboticsForward.py first to generate data.")
        return
    
    # Use the first non-empty dataset for time
    times = None
    for key, value in data.items():
        if isinstance(value, np.ndarray) and value.size > 0 and value.ndim == 2 and value.shape[1] >= 2:
            times = value[:, 0]
            break
    
    if times is None:
        print("No time data found.")
        return
    
    # Extract data
    verticalDisp_abs = data["verticalDisp"][:, 3] if isinstance(data["verticalDisp"], np.ndarray) and data["verticalDisp"].size > 0 else np.array([])
    verticalVel_abs = data["verticalVel"][:, 3] if isinstance(data["verticalVel"], np.ndarray) and data["verticalVel"].size > 0 else np.array([])
    theta1_abs = data["theta1"][:, 3] if isinstance(data["theta1"], np.ndarray) and data["theta1"].size > 0 else np.array([])
    theta2_abs = data["theta2"][:, 3] if isinstance(data["theta2"], np.ndarray) and data["theta2"].size > 0 else np.array([])
    theta3_abs = data["theta3"][:, 3] if isinstance(data["theta3"], np.ndarray) and data["theta3"].size > 0 else np.array([])
    omega1_abs = data["omega1"][:, 3] if isinstance(data["omega1"], np.ndarray) and data["omega1"].size > 0 else np.array([])
    omega2_abs = data["omega2"][:, 3] if isinstance(data["omega2"], np.ndarray) and data["omega2"].size > 0 else np.array([])
    omega3_abs = data["omega3"][:, 3] if isinstance(data["omega3"], np.ndarray) and data["omega3"].size > 0 else np.array([])
    
    # Calculate relative angles and velocities; convert degrees->radians for *_deg files
    theta1_rad = np.deg2rad(theta1_abs) if len(theta1_abs) > 0 else np.array([])
    theta2_rad = np.deg2rad(theta2_abs) if len(theta2_abs) > 0 else np.array([])
    theta3_rad = np.deg2rad(theta3_abs) if len(theta3_abs) > 0 else np.array([])

    theta1 = theta1_rad
    theta2 = theta2_rad - theta1_rad if len(theta2_rad) > 0 and len(theta1_rad) > 0 else np.array([])
    theta3 = theta3_rad - theta2_rad if len(theta3_rad) > 0 and len(theta2_rad) > 0 else np.array([])

    omega1 = omega1_abs
    omega2 = omega2_abs - omega1_abs if len(omega2_abs) > 0 and len(omega1_abs) > 0 else np.array([])
    omega3 = omega3_abs - omega2_abs if len(omega3_abs) > 0 and len(omega2_abs) > 0 else np.array([])
    
    # Calculate accelerations
    verticalAcc = np.gradient(verticalVel_abs, times) if len(verticalVel_abs) > 0 else np.array([])
    epsilon1 = np.gradient(omega1, times) if len(omega1) > 0 else np.array([])
    epsilon2 = np.gradient(omega2, times) if len(omega2) > 0 else np.array([])
    epsilon3 = np.gradient(omega3, times) if len(omega3) > 0 else np.array([])
    

    
    # Create plots
    print("Creating plots...")

    # Trajectory from theta sensors (angles over time)
    if len(theta1_abs) > 0 and len(theta2_abs) > 0 and len(theta3_abs) > 0:
        save_plot(times, [theta1_abs, theta2_abs, theta3_abs],
                  ["theta1 (deg)", "theta2 (deg)", "theta3 (deg)"],
                  "Joint angles trajectory (sensors)", "Angle (deg)", "trajectory_angles.png")

    # Compute X(t), Y(t) from sensor angles (q1=theta1, q2=theta2) per formula
    if len(theta1_rad) > 0 and len(theta2_rad) > 0:
        X = l1*np.cos(theta1_rad) + l2*np.cos(theta2_rad) + l3*np.cos(theta1_rad + 0.5*theta2_rad)
        Y = l1*np.sin(theta1_rad) + l2*np.sin(theta2_rad) + l3*np.sin(theta1_rad + 0.5*theta2_rad)
        save_plot(times, [X], ["X(t)"], "X coordinate from sensors", "X (m)", "x_of_t.png")
        save_plot(times, [Y], ["Y(t)"], "Y coordinate from sensors", "Y (m)", "y_of_t.png")
        # XY path
        save_xy_plot([ (X, Y) ], ["Sensor XY"], "End-effector trajectory (XY) from sensors", "trajectory_xy.png")
    
    # Velocity plots
    if len(verticalVel_abs) > 0:
        save_plot(times, [verticalVel_abs],
                  ["Simulated"], "Vertical Velocity", "Velocity (m/s)", 
                  "vertical_velocity.png")
    
    if len(omega1) > 0:
        save_plot(times, [omega1],
                  ["Simulated"], "Omega1", "Angular Velocity (rad/s)", 
                  "omega1.png")
    
    if len(omega2) > 0:
        save_plot(times, [omega2],
                  ["Simulated"], "Omega2", "Angular Velocity (rad/s)", 
                  "omega2.png")
    
    if len(omega3) > 0:
        save_plot(times, [omega3], ["Simulated"], "Omega3", 
                  "Angular Velocity (rad/s)", "omega3.png")
    
    # Acceleration plots
    if len(verticalAcc) > 0:
        save_plot(times, [verticalAcc],
                  ["Simulated"], "Vertical Acceleration", "Acceleration (m/s²)", 
                  "vertical_acceleration.png")
    
    if len(epsilon1) > 0:
        save_plot(times, [epsilon1],
                  ["Simulated"], "Epsilon1", "Angular Acceleration (rad/s²)", 
                  "epsilon1.png", ["-"])
    
    if len(epsilon2) > 0:
        save_plot(times, [epsilon2],
                  ["Simulated"], "Epsilon2", "Angular Acceleration (rad/s²)", 
                  "epsilon2.png", ["-"])
    
    if len(epsilon3) > 0:
        save_plot(times, [epsilon3], ["Simulated"], "Epsilon3", 
                  "Angular Acceleration (rad/s²)", "epsilon3.png")
    
    # Torque plots
    plot_torques(data.get("Torques", np.array([])), times)
    
    print("Plots created successfully!")

if __name__ == "__main__":
    main() 
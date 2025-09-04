import os
import numpy as np
import matplotlib.pyplot as plt
from constants import l1, l2, l3

# Resolve repo root from this file location
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.abspath(os.path.join(_THIS_DIR, os.pardir))
SENSOR_DIR = os.path.join(ROOT_DIR, 'sensor_outputs')
PLOTS_DIR = os.path.join(ROOT_DIR, 'plots')

def save_plot(x, y_list, labels, title, ylabel, filename, outdir=PLOTS_DIR):
    os.makedirs(outdir, exist_ok=True)
    plt.figure(figsize=(8, 6))
    for y, lbl in zip(y_list, labels):
        plt.plot(x, y, label=lbl)
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

def main():
    print("Loading sensor data...")
    
    # Load all sensor data directly from files
    verticalDisp = np.loadtxt(os.path.join(SENSOR_DIR, "verticalDisp.txt"), comments='#', delimiter=',')
    verticalVel = np.loadtxt(os.path.join(SENSOR_DIR, "verticalVel.txt"), comments='#', delimiter=',')
    theta1 = np.loadtxt(os.path.join(SENSOR_DIR, "theta1_deg.txt"), comments='#', delimiter=',')
    theta2 = np.loadtxt(os.path.join(SENSOR_DIR, "theta2_deg.txt"), comments='#', delimiter=',')
    theta3 = np.loadtxt(os.path.join(SENSOR_DIR, "theta3_deg.txt"), comments='#', delimiter=',')
    omega1 = np.loadtxt(os.path.join(SENSOR_DIR, "omega1_deg.txt"), comments='#', delimiter=',')
    omega2 = np.loadtxt(os.path.join(SENSOR_DIR, "omega2_deg.txt"), comments='#', delimiter=',')
    omega3 = np.loadtxt(os.path.join(SENSOR_DIR, "omega3_deg.txt"), comments='#', delimiter=',')
    
    # Load torques
    try:
        torques = np.loadtxt(os.path.join(SENSOR_DIR, "tau_ff.txt"), delimiter=',', comments='#')
    except:
        torques = np.loadtxt(os.path.join(SENSOR_DIR, "Torques.txt"), delimiter=',', comments='#')
    
    # Extract time and data columns
    times = verticalDisp[:, 0]
    verticalDisp_data = verticalDisp[:, 3]
    verticalVel_data = verticalVel[:, 3]
    theta1_data = theta1[:, 3]
    theta2_data = theta2[:, 3]
    theta3_data = theta3[:, 3]
    omega1_data = omega1[:, 3]
    omega2_data = omega2[:, 3]
    omega3_data = omega3[:, 3]
    
    # Convert angles to radians
    theta1_rad = np.deg2rad(theta1_data)
    theta2_rad = np.deg2rad(theta2_data)
    theta3_rad = np.deg2rad(theta3_data)
    
    # Calculate relative angles and velocities
    theta1_rel = theta1_rad
    theta2_rel = theta2_rad - theta1_rad
    theta3_rel = theta3_rad - theta2_rad
    
    omega1_rel = omega1_data
    omega2_rel = omega2_data - omega1_data
    omega3_rel = omega3_data - omega2_data
    
    # Calculate accelerations
    verticalAcc = np.gradient(verticalVel_data, times)
    epsilon1 = np.gradient(omega1_rel, times)
    epsilon2 = np.gradient(omega2_rel, times)
    epsilon3 = np.gradient(omega3_rel, times)
    
    print("Creating plots...")
    
    # Joint angles trajectory
    save_plot(times, [theta1_data, theta2_data, theta3_data],
              ["theta1 (deg)", "theta2 (deg)", "theta3 (deg)"],
              "Joint angles trajectory", "Angle (deg)", "trajectory_angles.png")
    
    # X(t), Y(t) from sensor angles
    X = l1*np.cos(theta1_rad) + l2*np.cos(theta2_rad) + l3*np.cos(theta1_rad + 0.5*theta2_rad)
    Y = l1*np.sin(theta1_rad) + l2*np.sin(theta2_rad) + l3*np.sin(theta1_rad + 0.5*theta2_rad)
    
    save_plot(times, [X], ["X(t)"], "X coordinate", "X (m)", "x_of_t.png")
    save_plot(times, [Y], ["Y(t)"], "Y coordinate", "Y (m)", "y_of_t.png")
    save_xy_plot([(X, Y)], ["End-effector"], "End-effector trajectory (XY)", "trajectory_xy.png")
    
    # Velocity plots
    save_plot(times, [verticalVel_data], ["Vertical Velocity"], 
              "Vertical Velocity", "Velocity (m/s)", "vertical_velocity.png")
    save_plot(times, [omega1_rel], ["Omega1"], 
              "Omega1", "Angular Velocity (rad/s)", "omega1.png")
    save_plot(times, [omega2_rel], ["Omega2"], 
              "Omega2", "Angular Velocity (rad/s)", "omega2.png")
    save_plot(times, [omega3_rel], ["Omega3"], 
              "Omega3", "Angular Velocity (rad/s)", "omega3.png")
    
    # Acceleration plots
    save_plot(times, [verticalAcc], ["Vertical Acceleration"], 
              "Vertical Acceleration", "Acceleration (m/s²)", "vertical_acceleration.png")
    save_plot(times, [epsilon1], ["Epsilon1"], 
              "Epsilon1", "Angular Acceleration (rad/s²)", "epsilon1.png")
    save_plot(times, [epsilon2], ["Epsilon2"], 
              "Epsilon2", "Angular Acceleration (rad/s²)", "epsilon2.png")
    save_plot(times, [epsilon3], ["Epsilon3"], 
              "Epsilon3", "Angular Acceleration (rad/s²)", "epsilon3.png")
    
    # Torque plots
    if torques.shape[1] > 1:
        t_tau = torques[:, 0]
        torque_data = [torques[:, i] for i in range(1, torques.shape[1])]
        labels = [f"τ{i}" for i in range(len(torque_data))]
        save_plot(t_tau, torque_data, labels, "Joint Torques", "Torque (N·m)", "tau_ff.png")
    
    print("Plots created successfully!")

if __name__ == "__main__":
    main() 
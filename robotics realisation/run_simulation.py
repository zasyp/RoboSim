#!/usr/bin/env python3
"""
Main script to run the robotics simulation and generate plots.
This script will:
1. Run the forward kinematics simulation
2. Generate plots from sensor data
"""

import os
import sys
import subprocess

def main():
    print("=== RoboSim Robotics Simulation ===")
    print()
    
    # Step 1: Run the forward kinematics simulation
    print("Step 1: Running forward kinematics simulation...")
    try:
        # Import and run roboticsForward
        import roboticsForward
        print("✓ Simulation completed successfully!")
    except Exception as e:
        print(f"✗ Error running simulation: {e}")
        return
    
    print()
    
    # Step 2: Generate plots
    print("Step 2: Generating plots from sensor data...")
    try:
        import plot_sensors
        print("✓ Plots generated successfully!")
    except Exception as e:
        print(f"✗ Error generating plots: {e}")
        return
    
    print()
    print("=== Simulation Complete ===")
    print("Check the 'plots' directory for generated graphs.")
    print("Check the 'sensor_outputs' directory for raw sensor data.")

if __name__ == "__main__":
    main() 
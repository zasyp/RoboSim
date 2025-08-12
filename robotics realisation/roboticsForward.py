import os
import numpy as np
import matplotlib.pyplot as plt
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
from exudyn.robotics.special import *
from scipy.signal import savgol_filter
from helpful.constants import *

# ========================================
# VISUALIZATION SETUP
# ========================================
visualisationBox = VRobotBase(graphicsData=[graphicsBodyBox])
visualisationCylinder = VRobotLink(graphicsData=[graphicsBodyCylinder])
visualisationLink1 = VRobotLink(graphicsData=[graphicsBody1])
visualisationLink2 = VRobotLink(graphicsData=[graphicsBody2])
visualisationLink3 = VRobotLink(graphicsData=[graphicsBody3])

# ========================================
# ROBOT CONFIGURATION
# ========================================
useKT = True
q0 = np.array([0, 0, 0, 0])  # Initial configuration

# Create robot base
baseBox = RobotBase(visualization=visualisationBox)

# Initialize robot with gravity and tool
robot = Robot(
    gravity=g,
    base=baseBox,
    tool=RobotTool(HT=HT_tool)
)

# Add robot links with proper parent-child hierarchy
robot.AddLink(RobotLink(
    mass=m_cyl,
    COM=com_cyl_global,
    inertia=inertiaTensorCylinder,
    jointType='Pz',  # Prismatic joint along z-axis
    parent=-1,  # Connected directly to base
    preHT=preHT_Cyl,
    visualization=visualisationCylinder,
    PDcontrol=(kp_trans, kd_trans)
))

robot.AddLink(RobotLink(
    mass=m1,
    COM=joint1_pos,
    inertia=inertiaTensor1,
    jointType='Rz',  # Revolute joint around z-axis
    parent=0,  # Child of cylinder (link 0)
    preHT=preHT_1,
    visualization=visualisationLink1,
    PDcontrol=(kp_rot, kd_rot)
))

robot.AddLink(RobotLink(
    mass=m2,
    COM=joint2_pos,
    inertia=inertiaTensor2,
    jointType='Rz',
    parent=1,  # Child of link 1
    preHT=preHT_2,
    visualization=visualisationLink2,
    PDcontrol=(kp_rot2, kd_rot2)
))

robot.AddLink(RobotLink(
    mass=m3,
    COM=joint3_pos,
    inertia=inertiaTensor3,
    jointType='Rz',
    parent=2,  # Child of link 2
    preHT=preHT_3,
    visualization=visualisationLink3,
    PDcontrol=(0, 0)  # No PD control for last link
))
# ========================================
# SYSTEM SETUP
# ========================================
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Create kinematic tree for the robot
robotDict = robot.CreateKinematicTree(
    mbs=mbs,
    name="WHR"
)
oKT = robotDict['objectKinematicTree']
nodeNumber = mbs.GetObject(oKT)['nodeNumber']

# Add coordinate connector constraint
mJoint2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=2))
mJoint3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=3))

mbs.AddObject(ObjectConnectorCoordinate(
    markerNumbers=[mJoint3, mJoint2],
    factorValue1=-0.5,
))

# ========================================
# TRAJECTORY DEFINITION
# ========================================
robotTrajectory = Trajectory(initialCoordinates=q0, initialTime=0)

# Define waypoints for the trajectory
q1 = [0.1, -0.5 * np.pi, 0.3 * np.pi, 0]
q2 = [0.2, 0.5 * np.pi, -0.3 * np.pi, 0]
q3 = [0.1, -0.5 * np.pi, -0.1 * np.pi, 0]
q4 = [0.3, -0.3 * np.pi, -0.4 * np.pi, 0]
q5 = [0, 0, 0, 0]

# Add motion profiles with constant acceleration
robotTrajectory.Add(ProfileConstantAcceleration(q1, 1))
robotTrajectory.Add(ProfileConstantAcceleration(q2, 1))
robotTrajectory.Add(ProfileConstantAcceleration(q3, 1))
robotTrajectory.Add(ProfileConstantAcceleration(q4, 1))
robotTrajectory.Add(ProfileConstantAcceleration(q5, 1))
# ==========================================
# PRE-STEP USER FUNCTION FOR DYNAMIC CONTROL
# ==========================================
torque_values = []
def PreStepUF(mbs, t):
    if useKT:
        # Getting trajectory parameters
        [u,v,a] = robotTrajectory.Evaluate(t)
        # Calculating torques according to tau = M * ddq
        HT = robot.JointHT(u)
        jointJacs = JointJacobian(robot, HT, HT)
        MM = MassMatrix(robot, HT, jointJacs)
        dynamical = MM.dot(a)
        torque_values.append(dynamical)
        # Setting system parameters
        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
        mbs.SetObjectParameter(oKT, 'jointForceVector', dynamical)
    return True

mbs.SetPreStepUserFunction(PreStepUF)

# ========================================
# SENSORS
# ========================================
output_dir = "sensor_outputs"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)
verticalDispSens = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=0,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.Displacement,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "verticalDisp.txt"),
    name="verticalDisp"
))
verticalVelSens = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=0,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.VelocityLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "verticalVel.txt"),
    name="verticalVel"
))
# Joint 1 sensors (Rz)
theta1Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=1,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.Rotation,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "theta1_deg.txt"),
    name="theta1_deg"
))
omega1Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=1,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "omega1_deg.txt"),
    name="omega1_deg"
))
# Joint 2 sensors (Rz)
theta2Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=2,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.Rotation,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "theta2_deg.txt"),
    name="theta2_deg"
))
omega2Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=2,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "omega2_deg.txt"),
    name="omega2_deg"
))
# Joint 3 sensors (Rz)
theta3Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=3,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.Rotation,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "theta3_deg.txt"),
    name="theta3_deg"
))
omega3Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=3,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.AngularVelocityLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "omega3_deg.txt"),
    name="omega3_deg"
))

# ========================================
# SIMULATION SETUP
# ========================================
mbs.Assemble()
simulationSettings = exu.SimulationSettings()

# Time integration settings
tEnd = 5  # Simulation time [s]
h = 1e-2  # Step size [s]

simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1

# Output settings
simulationSettings.solutionSettings.solutionWritePeriod = 0.005  # 5 ms
simulationSettings.solutionSettings.sensorsWritePeriod = 0.005

# Visualization settings
SC.visualizationSettings.window.renderWindowSize = (1600, 1200)
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.general.drawWorldBasis = True
SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
SC.visualizationSettings.openGL.lineWidth = 2
SC.visualizationSettings.openGL.light0position = (-6, 2, 12, 0)

# Start renderer
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])

# Wait for user input before starting simulation
mbs.WaitForUserToContinue()

# Run dynamic simulation
mbs.SolveDynamic(
    simulationSettings=simulationSettings,
    solverType=exu.DynamicSolverType.TrapezoidalIndex2
)

# ========================================
# DATA PROCESSING AND PLOTTING (fixed)
# ========================================
verticalDisp_data = mbs.GetSensorStoredData(verticalDispSens)
theta1_data = mbs.GetSensorStoredData(theta1Sensor)
theta2_data = mbs.GetSensorStoredData(theta2Sensor)
theta3_data = mbs.GetSensorStoredData(theta3Sensor)
verticalVel_data = mbs.GetSensorStoredData(verticalVelSens)
omega1_data = mbs.GetSensorStoredData(omega1Sensor)
omega2_data = mbs.GetSensorStoredData(omega2Sensor)
omega3_data = mbs.GetSensorStoredData(omega3Sensor)

times = theta1_data[:, 0]
verticalDisp_abs = verticalDisp_data[:, 3]
verticalVel_abs = verticalVel_data[:, 3]
theta1_abs = theta1_data[:, 3]
theta2_abs = theta2_data[:, 3]
theta3_abs = theta3_data[:, 3]
omega1_abs = omega1_data[:, 3]
omega2_abs = omega2_data[:, 3]
omega3_abs = omega3_data[:, 3]

theta1 = theta1_abs.copy()
theta2 = theta2_abs - theta1_abs
theta3 = theta3_abs - theta2_abs
verticalVel = verticalVel_abs
omega1 = omega1_abs.copy()
omega2 = omega2_abs - omega1_abs
omega3 = omega3_abs - omega2_abs
verticalAcc = np.gradient(verticalVel, times)
epsilon1 = np.gradient(omega1, times)
epsilon2 = np.gradient(omega2, times)
epsilon3 = np.gradient(omega3, times)

n = len(times)
ideal_positions = np.zeros((n, 4))
ideal_velocities = np.zeros((n, 4))
ideal_accelerations = np.zeros((n, 4))
for i, t in enumerate(times):
    u, v, a = robotTrajectory.Evaluate(t)
    ideal_positions[i] = u
    ideal_velocities[i] = v
    ideal_accelerations[i] = a

# Adjust ideal vertical position to match initial condition
z0 = verticalDisp_data[0, 3]
ideal_vertical_position = z0 + ideal_positions[:, 0]

# ========================================
# MAIN PLOTS
# ========================================
def plot_all_results(times,
                     verticalDisp, ideal_vertical_position,
                     theta1, theta2, theta3, ideal_positions,
                     verticalVel, omega1, omega2, omega3, ideal_velocities,
                     verticalAcc, epsilon1, epsilon2, epsilon3, ideal_accelerations,
                     torque_times, torques,
                     output_dir="plots"):
    os.makedirs(output_dir, exist_ok=True)

    torques = np.array(torques)
    torque_times = np.linspace(0, 10, len(torques))

    def save_plot(x, y_list, labels, title, ylabel, filename, linestyles=None):
        plt.figure(figsize=(8, 6))
        for i, (y, label) in enumerate(zip(y_list, labels)):
            if linestyles is not None and i < len(linestyles):
                plt.plot(x, y, label=label, linestyle=linestyles[i])
            else:
                plt.plot(x, y, label=label)
        plt.title(title)
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True)
        plt.tight_layout()
        plt.savefig(os.path.join(output_dir, filename), dpi=300)
        plt.close()

    step = 1

    # Velocities
    save_plot(times, [verticalVel, ideal_velocities[:, 0]],
              ["Simulated", "Ideal"], "Vertical Velocity", "Velocity (m/s)",
              "vertical_velocity.png", linestyles=['-', '--'])
    save_plot(times, [omega1, ideal_velocities[:, 1]],
              ["Simulated", "Ideal"], "Omega1", "Angular Velocity (rad/s)",
              "omega1.png", linestyles=['-', '--'])
    save_plot(times, [omega2, ideal_velocities[:, 2]],
              ["Simulated", "Ideal"], "Omega2", "Angular Velocity (rad/s)",
              "omega2.png", linestyles=['-', '--'])
    save_plot(times, [omega3], ["Simulated"], "Omega3", "Angular Velocity (rad/s)",
              "omega3.png", linestyles=['-'])

    # Velocity error plots
    save_plot(times, [ideal_velocities[:, 0] - verticalVel],
              ["Error"], "Vertical Velocity Error", "Error (m/s)",
              "err_vertical_velocity.png")
    save_plot(times, [ideal_velocities[:, 1] - omega1],
              ["Error"], "Omega1 Error", "Error (rad/s)",
              "err_omega1.png")
    save_plot(times, [ideal_velocities[:, 2] - omega2],
              ["Error"], "Omega2 Error", "Error (rad/s)",
              "err_omega2.png")

    # Accelerations
    save_plot(times[::step], [verticalAcc[::step], ideal_accelerations[::step, 0]],
              ["Simulated", "Ideal"], "Vertical Acceleration", "Acceleration (m/s²)",
              "vertical_acceleration.png", linestyles=['-', '--'])
    save_plot(times[::step], [epsilon1[::step], ideal_accelerations[::step, 1]],
              ["Simulated", "Ideal"], "Epsilon1", "Angular Acceleration (rad/s²)",
              "epsilon1.png", linestyles=['-', '--'])
    save_plot(times[::step], [epsilon2[::step], ideal_accelerations[::step, 2]],
              ["Simulated", "Ideal"], "Epsilon2", "Angular Acceleration (rad/s²)",
              "epsilon2.png", linestyles=['-', '--'])
    save_plot(times[::step], [epsilon3[::step]], ["Simulated"], "Epsilon3",
              "Angular Acceleration (rad/s²)", "epsilon3.png", linestyles=['-'])
    # Acceleration errors
    save_plot(times[::step], [ideal_accelerations[::step, 0] - verticalAcc[::step]],
              ["Error"], "Vertical Acceleration Error", "Error (m/s²)",
              "err_vertical_acceleration.png")
    save_plot(times[::step], [ideal_accelerations[::step, 1] - epsilon1[::step]],
              ["Error"], "Epsilon1 Error", "Error (rad/s²)", "err_epsilon1.png")
    save_plot(times[::step], [ideal_accelerations[::step, 2] - epsilon2[::step]],
              ["Error"], "Epsilon2 Error", "Error (rad/s²)", "err_epsilon2.png")

    # Torques
    save_plot(torque_times, [torques[:, 0]], ["Cylinder Force"],
              "Cylinder Actuation Force (Pz joint)", "Force (N)",
              "cylinder_force.png")

    save_plot(torque_times, [torques[:, 1], torques[:, 2], torques[:, 3]],
              ["Link 1 (Rz)", "Link 2 (Rz)", "Link 3 (Rz)"],
              "Joint Actuation Torques", "Torque (N·m)", "all_torques.png")


plot_all_results(
    times,
    verticalDisp_abs, ideal_vertical_position,
    theta1, theta2, theta3, ideal_positions,
    verticalVel, omega1, omega2, omega3, ideal_velocities,
    verticalAcc, epsilon1, epsilon2, epsilon3, ideal_accelerations,
    times, torque_values
)

# Cleanup
exu.StopRenderer()
mbs.SolutionViewer()

# ========================================
# SCATTER PLOTS FOR ACCELERATIONS
# ========================================
def plot_accelerations_scatter(times,
                               verticalAcc, ideal_vertical_acc,
                               epsilon1, epsilon2, epsilon3,
                               ideal_eps1, ideal_eps2, ideal_eps3,
                               output_dir="plots"):
    os.makedirs(output_dir, exist_ok=True)

    def save_scatter(x, y_list, labels, title, ylabel, filename, alpha=0.6, s=10):
        plt.figure(figsize=(8, 6))
        for y, label in y_list:
            plt.scatter(x, y, label=label, alpha=alpha, s=s)
        plt.title(title)
        plt.xlabel("Time (s)")
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.tight_layout()
        plt.show()
        plt.savefig(os.path.join(output_dir, filename), dpi=300)
        plt.close()

    step = 1  # Use every point or increase step to reduce density
    t_sub = times[::step]
    verticalAcc_sub = verticalAcc[::step]
    epsilon1_sub = epsilon1[::step]
    epsilon2_sub = epsilon2[::step]
    epsilon3_sub = epsilon3[::step]

    ideal_vertical_acc_sub = ideal_vertical_acc[::step]
    ideal_eps1_sub = ideal_eps1[::step]
    ideal_eps2_sub = ideal_eps2[::step]
    ideal_eps3_sub = ideal_eps3[::step]

    # Vertical acceleration
    save_scatter(t_sub,
                 [(verticalAcc_sub, "Simulated"), (ideal_vertical_acc_sub, "Ideal")],
                 ["Simulated", "Ideal"],
                 "Vertical Acceleration (Scatter)", "Acceleration (m/s²)",
                 "vertical_acceleration_scatter.png")

    # Epsilon1
    save_scatter(t_sub,
                 [(epsilon1_sub, "Simulated")],
                 ["Simulated", "Ideal"],
                 "Epsilon1 - Angular Acceleration Link 1 (Scatter)", "Angular Acceleration (rad/s²)",
                 "epsilon1_scatter.png")

    # Epsilon2
    save_scatter(t_sub,
                 [(epsilon2_sub, "Simulated")],
                 ["Simulated", "Ideal"],
                 "Epsilon2 - Angular Acceleration Link 2 (Scatter)", "Angular Acceleration (rad/s²)",
                 "epsilon2_scatter.png")

    # Epsilon3 — only simulated (no ideal torque applied)
    save_scatter(t_sub,
                 [(epsilon3_sub, "Simulated")],
                 ["Simulated"],
                 "Epsilon3 - Angular Acceleration Link 3 (Scatter)", "Angular Acceleration (rad/s²)",
                 "epsilon3_scatter.png")

ideal_vertical_acc = ideal_accelerations[:, 0]
ideal_eps1 = ideal_accelerations[:, 1]
ideal_eps2 = ideal_accelerations[:, 2]
ideal_eps3 = ideal_accelerations[:, 3]

plot_accelerations_scatter(
    times,
    verticalAcc, ideal_vertical_acc,
    epsilon1, epsilon2, epsilon3,
    ideal_eps1, ideal_eps2, ideal_eps3,
    output_dir="plots"
)

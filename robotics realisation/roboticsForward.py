import os
import numpy as np
import matplotlib.pyplot as plt
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
from exudyn.robotics.special import *
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
robotTrajectory.Add(ProfileConstantAcceleration(q1, 2))
robotTrajectory.Add(ProfileConstantAcceleration(q2, 2))
robotTrajectory.Add(ProfileConstantAcceleration(q3, 2))
robotTrajectory.Add(ProfileConstantAcceleration(q4, 2))
robotTrajectory.Add(ProfileConstantAcceleration(q5, 2))

# ========================================
# SENSORS
# ========================================
output_dir = "sensor_outputs"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# Vertical motion sensors (cylinder - Pz joint)
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

verticalAccSens = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=0,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.AccelerationLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "verticalAcc.txt"),
    name="verticalAcc"
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

epsilon1Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=1,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.AngularAccelerationLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "epsilon1_deg.txt"),
    name="epsilon1_deg"
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

epsilon2Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=2,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.AngularAccelerationLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "epsilon2_deg.txt"),
    name="epsilon2_deg"
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

epsilon3Sensor = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT,
    linkNumber=3,
    localPosition=[0, 0, 0],
    outputVariableType=exu.OutputVariableType.AngularAccelerationLocal,
    storeInternal=True,
    writeToFile=True,
    fileName=os.path.join(output_dir, "epsilon3_deg.txt"),
    name="epsilon3_deg"
))

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

mbs.SetPreStepUserFunction(PreStepUF)

# ========================================
# SIMULATION SETUP
# ========================================
mbs.Assemble()

simulationSettings = exu.SimulationSettings()

# Time integration settings
tEnd = 10  # Simulation time [s]
h = 0.1e-3  # Step size [s]

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
# DATA PROCESSING AND PLOTTING
# ========================================
# Load sensor data
verticalDisp_data = mbs.GetSensorStoredData(verticalDispSens)
theta1_data = mbs.GetSensorStoredData(theta1Sensor)
theta2_data = mbs.GetSensorStoredData(theta2Sensor)
theta3_data = mbs.GetSensorStoredData(theta3Sensor)

verticalVel_data = mbs.GetSensorStoredData(verticalVelSens)
omega1_data = mbs.GetSensorStoredData(omega1Sensor)
omega2_data = mbs.GetSensorStoredData(omega2Sensor)
omega3_data = mbs.GetSensorStoredData(omega3Sensor)

verticalAcc_data = mbs.GetSensorStoredData(verticalAccSens)
epsilon1_data = mbs.GetSensorStoredData(epsilon1Sensor)
epsilon2_data = mbs.GetSensorStoredData(epsilon2Sensor)
epsilon3_data = mbs.GetSensorStoredData(epsilon3Sensor)

# Extract time vector
times = theta1_data[:, 0]

# Extract actual sensor data (relative angles)
verticalDisp = verticalDisp_data[:, 3]  # z-component
theta1 = theta1_data[:, 3]
theta2 = theta2_data[:, 3] - theta1
theta3 = theta3_data[:, 3] - theta2 - theta1

verticalVel = verticalVel_data[:, 3]
omega1 = omega1_data[:, 3]
omega2 = omega2_data[:, 3] - omega1
omega3 = omega3_data[:, 3] - omega2 - omega1

# Calculate accelerations via numerical differentiation
verticalAcc_calc = np.gradient(verticalVel, times)
epsilon1_calc = np.gradient(omega1, times)
epsilon2_calc = np.gradient(omega2, times)
epsilon3_calc = np.gradient(omega3, times)

# Generate ideal trajectory data
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
# 3x4 grid of sensor data
plt.figure(figsize=(20, 15))

# Position plots
plt.subplot(3, 4, 1)
plt.plot(times, verticalDisp, 'b-', label='Actual')
plt.plot(times, ideal_vertical_position, 'r--', label='Ideal')
plt.title('Vertical Position (m)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 2)
plt.plot(times, theta1, 'b-', label='Actual')
plt.plot(times, ideal_positions[:, 1], 'r--', label='Ideal')
plt.title('Theta1 (rad)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 3)
plt.plot(times, theta2, 'b-', label='Actual')
plt.plot(times, ideal_positions[:, 2], 'r--', label='Ideal')
plt.title('Theta2 (rad)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 4)
plt.plot(times, theta3, 'b-', label='Actual')
plt.plot(times, ideal_positions[:, 3], 'r--', label='Ideal')
plt.title('Theta3 (rad)')
plt.legend()
plt.grid()

# Velocity plots
plt.subplot(3, 4, 5)
plt.plot(times, verticalVel, 'b-', label='Actual')
plt.plot(times, ideal_velocities[:, 0], 'r--', label='Ideal')
plt.title('Vertical Velocity (m/s)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 6)
plt.plot(times, omega1, 'b-', label='Actual')
plt.plot(times, ideal_velocities[:, 1], 'r--', label='Ideal')
plt.title('Omega1 (rad/s)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 7)
plt.plot(times, omega2, 'b-', label='Actual')
plt.plot(times, ideal_velocities[:, 2], 'r--', label='Ideal')
plt.title('Omega2 (rad/s)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 8)
plt.plot(times, omega3, 'b-', label='Actual')
plt.plot(times, ideal_velocities[:, 3], 'r--', label='Ideal')
plt.title('Omega3 (rad/s)')
plt.legend()
plt.grid()

# Acceleration plots
plt.subplot(3, 4, 9)
plt.plot(times, verticalAcc_calc, 'b-', label='Calculated')
plt.plot(times, ideal_accelerations[:, 0], 'r--', label='Ideal')
plt.title('Vertical Acceleration (m/s²)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 10)
plt.plot(times, epsilon1_calc, 'b-', label='Calculated')
plt.plot(times, ideal_accelerations[:, 1], 'r--', label='Ideal')
plt.title('Epsilon1 (rad/s²)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 11)
plt.plot(times, epsilon2_calc, 'b-', label='Calculated')
plt.plot(times, ideal_accelerations[:, 2], 'r--', label='Ideal')
plt.title('Epsilon2 (rad/s²)')
plt.legend()
plt.grid()

plt.subplot(3, 4, 12)
plt.plot(times, epsilon3_calc, 'b-', label='Calculated')
plt.plot(times, ideal_accelerations[:, 3], 'r--', label='Ideal')
plt.title('Epsilon3 (rad/s²)')
plt.legend()
plt.grid()

plt.tight_layout(pad=2.0)
plt.savefig('all_sensors_data_with_ideal.png', dpi=300)
plt.close()

# ========================================
# ERROR PLOTS
# ========================================
plt.figure(figsize=(15, 15))

# Position errors
plt.subplot(3, 4, 1)
plt.plot(times, ideal_vertical_position - verticalDisp)
plt.title('Vertical Position Error')
plt.ylabel('Error (m)')
plt.grid()

plt.subplot(3, 4, 2)
plt.plot(times, ideal_positions[:, 1] - theta1)
plt.title('Theta1 Error')
plt.ylabel('Error (rad)')
plt.grid()

plt.subplot(3, 4, 3)
plt.plot(times, ideal_positions[:, 2] - theta2)
plt.title('Theta2 Error')
plt.ylabel('Error (rad)')
plt.grid()

# Velocity errors
plt.subplot(3, 4, 5)
plt.plot(times, ideal_velocities[:, 0] - verticalVel)
plt.title('Vertical Velocity Error')
plt.ylabel('Error (m/s)')
plt.grid()

plt.subplot(3, 4, 6)
plt.plot(times, ideal_velocities[:, 1] - omega1)
plt.title('Omega1 Error')
plt.ylabel('Error (rad/s)')
plt.grid()

plt.subplot(3, 4, 7)
plt.plot(times, ideal_velocities[:, 2] - omega2)
plt.title('Omega2 Error')
plt.ylabel('Error (rad/s)')
plt.grid()

# Acceleration errors
plt.subplot(3, 4, 9)
plt.plot(times[1::3], ideal_accelerations[1::3, 0] - verticalAcc_calc[0:-1:3])
plt.title('Vertical Acceleration Error')
plt.ylabel('Error (m/s²)')
plt.xlabel('Time (s)')
plt.grid()

plt.subplot(3, 4, 10)
plt.plot(times, ideal_accelerations[:, 1] - epsilon1_calc)
plt.title('Epsilon1 Error')
plt.ylabel('Error (rad/s²)')
plt.xlabel('Time (s)')
plt.grid()

plt.subplot(3, 4, 11)
plt.plot(times, ideal_accelerations[:, 2] - epsilon2_calc)
plt.title('Epsilon2 Error')
plt.ylabel('Error (rad/s²)')
plt.xlabel('Time (s)')
plt.grid()

plt.savefig('all_errors.png', dpi=300)
plt.close()

# ========================================
# TORQUE PLOTS
# ========================================
# Convert torque_values list to numpy array
torques = np.array(torque_values)

# Create proper time vector for torque data
# Since torque_values is recorded at every simulation step
torque_times = np.linspace(0, tEnd, len(torques))

# Save torque data
with open("sensor_outputs/Torques.txt", "w") as f:
    for tau in torque_values:
        f.write(str(tau) + "\n")

# Separate plot for cylinder force (Pz joint)
plt.figure(figsize=(12, 6))
plt.plot(torque_times, torques[:, 0], 'b-', linewidth=2.5, label='Cylinder Force')
plt.title('Cylinder Actuation Force (Pz joint)', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Force (N)', fontsize=12)
plt.legend(fontsize=12)
plt.grid(True, alpha=0.7)
plt.tight_layout()
plt.savefig('cylinder_force.png', dpi=300)
plt.close()

# Combined torque plot
plt.figure(figsize=(12, 8))
plt.plot(torque_times, torques[:, 1], label='Link 1 (Rz)')
plt.plot(torque_times, torques[:, 2], label='Link 2 (Rz)')
plt.plot(torque_times, torques[:, 3], label='Link 3 (Rz)')

plt.title('Joint Actuation Torques', fontsize=14)
plt.xlabel('Time (s)', fontsize=12)
plt.ylabel('Torque (N·m)', fontsize=12)
plt.legend()
plt.grid(True, alpha=0.7)
plt.tight_layout()
plt.savefig('all_torques.png', dpi=300)
plt.show()

# Cleanup
exu.StopRenderer()
mbs.SolutionViewer()
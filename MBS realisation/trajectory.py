import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np
from exudyn.robotics.motion import Trajectory, ProfilePTP
from helpful.constants import *

# Initial and final coordinates
q0 = [0, 0, 0]  # Including all four joints
q1 = [0.2, -2*np.pi/3, -2*np.pi/3]

# Initialize SystemContainer and MainSystem
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Ground and bodies
oGround = mbs.CreateGround(referencePosition=[0, -145/1000, -805/1000], graphicsDataList=[box_graphics])

i1 = RigidBodyInertia(mass=m1, inertiaTensor=inertiaTensor1)
i2 = RigidBodyInertia(mass=m2, inertiaTensor=inertiaTensor2)
i3 = RigidBodyInertia(mass=m3, inertiaTensor=inertiaTensor3)
iCilinder = RigidBodyInertia(mass=m_cil, inertiaTensor=inertiaTensorCilinder)

[n0, b0] = AddRigidBody(
    mainSys=mbs,
    inertia=iCilinder,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=com_cil_global,
    rotationMatrix=np.eye(3),
    gravity=g,
    graphicsDataList=[graphicsBodyCilinder]
)
[n1, b1] = AddRigidBody(
    mainSys=mbs,
    inertia=i1,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=com1_global,
    rotationMatrix=np.eye(3),
    gravity=g,
    graphicsDataList=[graphicsBody1]
)
[n2, b2] = AddRigidBody(
    mainSys=mbs,
    inertia=i2,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=com2_global,
    rotationMatrix=np.eye(3),
    gravity=g,
    graphicsDataList=[graphicsBody2]
)
[n3, b3] = AddRigidBody(
    mainSys=mbs,
    inertia=i3,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=com3_global,
    rotationMatrix=np.eye(3),
    gravity=g,
    graphicsDataList=[graphicsBody3]
)

# Markers
link0_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))  # Z-position for prismatic joint
link1_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n1, coordinate=6))  # Rotation Z for revolute joint 1
link2_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n2, coordinate=6))  # Rotation Z for revolute joint 2
link3_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n3, coordinate=6))  # Rotation Z for revolute joint 3

markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0]))
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=joint0_pos))

# Joints
jointPrismatic = mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=joint0_pos,
                                          useGlobalFrame=True, axis=[0, 0, 1],  # Z-axis
                                          axisRadius=0.2*w, axisLength=300/1000)
jointRevolute1 = mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], position=joint1_pos,
                                         axis=[0, 0, 1], axisRadius=0.2*w, axisLength=1*w)
jointRevolute2 = mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=joint2_pos,
                                         axis=[0, 0, 1], axisRadius=0.2*w, axisLength=1*w)
jointRevolute3 = mbs.CreateRevoluteJoint(bodyNumbers=[b2, b3], position=joint3_pos,
                                         axis=[0, 0, 1], axisRadius=0.2*w, axisLength=0.3*w)

# Simulation settings
simulationSettings = exu.SimulationSettings()
tEnd = 3
h = 1e-4
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005

# Visualization settings
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

# Sensors
zpos_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b0,
                                     localPosition=joint0_pos, fileName='../solution/sensorPos0.txt',
                         outputVariableType=exu.OutputVariableType.Position))
zvel_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b0,
                                     localPosition=joint0_pos, fileName='../solution/sensorVelPos0.txt',
                         outputVariableType=exu.OutputVariableType.Velocity))

theta1_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b1,
                                       localPosition=joint1_pos, fileName='../solution/sensorPos1.txt',
                         outputVariableType=exu.OutputVariableType.Rotation))
omega1_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b1,
                                       localPosition=joint1_pos, fileName='../solution/sensorVel1.txt',
                         outputVariableType=exu.OutputVariableType.AngularVelocity))

theta2_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b2,
                                       localPosition=joint2_pos, fileName='../solution/sensorPos2.txt',
                         outputVariableType=exu.OutputVariableType.Rotation))
omega2_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b2,
                                       localPosition=joint2_pos, fileName='../solution/sensorVel2.txt',
                         outputVariableType=exu.OutputVariableType.AngularVelocity))

theta3_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b3,
                                       localPosition=joint3_pos, fileName='../solution/sensorPos3.txt',
                         outputVariableType=exu.OutputVariableType.Rotation))
omega3_sens = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=b3,
                                       localPosition=joint3_pos, fileName='../solution/sensorVel3.txt',
                         outputVariableType=exu.OutputVariableType.AngularVelocity))

# Trajectory profiles
traj_d = Trajectory(initialCoordinates=[0], initialTime=0)
traj_d.Add(ProfilePTP([q1[0]], maxVelocities=[1.0], maxAccelerations=[2.0], syncAccTimes=False))

traj_theta1 = Trajectory(initialCoordinates=[0], initialTime=0)
traj_theta1.Add(ProfilePTP([q1[1]], maxVelocities=[10.0], maxAccelerations=[10.0], syncAccTimes=False))

traj_theta2 = Trajectory(initialCoordinates=[q1[1]], initialTime=0)
traj_theta2.Add(ProfilePTP([q1[1]+q1[2]], maxVelocities=[10.0], maxAccelerations=[10.0], syncAccTimes=False))

traj_theta3 = Trajectory(initialCoordinates=[q1[1]+q1[2]], initialTime=0)
traj_theta3.Add(ProfilePTP([q1[1]+q1[2]/2], maxVelocities=[10.0], maxAccelerations=[10.0], syncAccTimes=False))

# PID coefficients
Kp_prismatic = 1000
Ki_prismatic = 1000
Kd_prismatic = 700

Kp_revolute1 = 0
Ki_revolute1 = 50
Kd_revolute1 = 415

Kp_revolute2 = 400
Ki_revolute2 = 50
Kd_revolute2 = 100

Kp_revolute3 = 400
Ki_revolute3 = 50
Kd_revolute3 = 100

# Markers for loads
markerBody0_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0, 0, 0]))
markerBody1_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0, 0, 0]))
markerBody2_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[0, 0, 0]))
markerBody3_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b3, localPosition=[0, 0, 0]))

# Global variables for integrals
integral_d = 0
integral_theta1 = 0
integral_theta2 = 0
integral_theta3 = 0
t_prev = 0

# Control functions
def ForceControlZ(mbs, t, loadVector):
    global integral_d, t_prev
    positions, velocities, _ = traj_d.Evaluate(t)
    d_des = positions[0]
    d_des_v = velocities[0]
    pos = mbs.GetNodeOutput(n0, exu.OutputVariableType.Position)
    vel = mbs.GetNodeOutput(n0, exu.OutputVariableType.Velocity)
    d_initial = com_cil_global[2]
    d_curr = pos[2] - d_initial
    d_vel = vel[2]
    error = d_des - d_curr
    dt = t - t_prev
    if dt > 0:
        integral_d += error * dt
    t_prev = t
    F = Kp_prismatic * error + Ki_prismatic * integral_d + Kd_prismatic * (d_des_v - d_vel)
    return [0, 0, F]

def TorqueControlRevolute1(mbs, t, loadVector):
    global integral_theta1, t_prev
    positions, velocities, _ = traj_theta1.Evaluate(t)
    theta_des = positions[0]
    theta_des_v = velocities[0]
    rot = mbs.GetNodeOutput(n1, exu.OutputVariableType.Rotation)
    theta_curr = rot[2]
    omega_curr = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)[2]
    error = theta_des - theta_curr
    dt = t - t_prev
    if dt > 0:
        integral_theta1 += error * dt
    t_prev = t
    T = Kp_revolute1 * error + Ki_revolute1 * integral_theta1 + Kd_revolute1 * (theta_des_v - omega_curr)
    return [0, 0, T]

def TorqueControlRevolute2(mbs, t, loadVector):
    global integral_theta1, t_prev
    positions, velocities, _ = traj_theta1.Evaluate(t)
    theta_des = positions[0]
    theta_des_v = velocities[0]
    rot = mbs.GetNodeOutput(n1, exu.OutputVariableType.Rotation)
    theta_curr = rot[2]
    omega_curr = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)[2]
    error = theta_des - theta_curr
    dt = t - t_prev
    if dt > 0:
        integral_theta1 += error * dt
    t_prev = t
    T = Kp_revolute1 * error + Ki_revolute1 * integral_theta1 + Kd_revolute1 * (theta_des_v - omega_curr)
    return [0, 0, T]

def TorqueControlRevolute3(mbs, t, loadVector):
    global integral_theta3, t_prev
    positions, velocities, _ = traj_theta3.Evaluate(t)
    theta_des = positions[0]
    theta_des_v = velocities[0]
    rot = mbs.GetNodeOutput(n3, exu.OutputVariableType.Rotation)
    theta_curr = rot[2]
    omega_curr = mbs.GetNodeOutput(n3, exu.OutputVariableType.AngularVelocity)[2]
    error = theta_des - theta_curr
    dt = t - t_prev
    if dt > 0:
        integral_theta3 += error * dt
    t_prev = t
    T = Kp_revolute3 * error + Ki_revolute3 * integral_theta3 + Kd_revolute3 * (theta_des_v - omega_curr)
    return [0, 0, T]

# Add loads
loadForceZ = mbs.AddLoad(LoadForceVector(
    markerNumber=markerBody0_com,
    loadVector=[0, 0, 0],
    bodyFixed=False,
    loadVectorUserFunction=ForceControlZ
))

loadTorque1 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=markerBody1_com,
    loadVector=[0, 0, 0],
    bodyFixed=False,
    loadVectorUserFunction=TorqueControlRevolute1
))

loadTorque2 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=markerBody2_com,
    loadVector=[0, 0, 0],
    bodyFixed=False,
    loadVectorUserFunction=TorqueControlRevolute2
))

loadTorque3 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=markerBody3_com,
    loadVector=[0, 0, 0],
    bodyFixed=False,
    loadVectorUserFunction=TorqueControlRevolute3
))

# Assemble and simulate
mbs.Assemble()
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])
SC.renderer.DoIdleTasks()

link1_grpah_data = mbs.GetSensorStoredData(theta1_sens)
link2_grpah_data = mbs.GetSensorStoredData(theta2_sens)

mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()
mbs.SolutionViewer()

# Prismatic graph (Z-axis)
mbs.PlotSensor(sensorNumbers=[zpos_sens], components=[2],
               title='Position of Prismatic Joint (Z)',
               yLabel='Position [m]',
               )

mbs.PlotSensor(sensorNumbers=[zvel_sens], components=[2],
               title='Velocity of Prismatic Joint (Z)',
               yLabel='Velocity [m/s]',
               figSize=(10, 4))

# Revolute graphs
for i, (pos_sens, vel_sens) in enumerate([(theta1_sens, omega1_sens),
                                          (theta2_sens, omega2_sens),
                                          (theta3_sens, omega3_sens)]):
    mbs.PlotSensor(sensorNumbers=[pos_sens], components=[2],
                   title=f'Rotation of Joint {i + 1} (Z)',
                   yLabel=f'Theta_{i + 1} [rad]',
                   figSize=(10, 4))

    mbs.PlotSensor(sensorNumbers=[vel_sens], components=[2],
                   title=f'Angular Velocity of Joint {i + 1} (Z)',
                   yLabel=f'Omega_{i + 1} [rad/s]',
                   figSize=(10, 4))

import matplotlib.pyplot as plt

fig, axs = plt.subplots(3, 2, figsize=(12, 10), tight_layout=True)

sensors = {
    'Prismatic Position': (zpos_sens, 2, 'Position [m]'),
    'Prismatic Velocity': (zvel_sens, 2, 'Velocity [m/s]'),
    'Joint1 Rotation': (theta1_sens, 2, 'Rotation [rad]'),
    'Joint1 Velocity': (omega1_sens, 2, 'Angular Velocity [rad/s]'),
    'Joint2 Rotation': (theta2_sens, 2, 'Rotation [rad]'),
    'Joint2 Velocity': (omega2_sens, 2, 'Angular Velocity [rad/s]'),
    'Joint3 Rotation': (theta3_sens, 2, 'Rotation [rad]'),
    'Joint3 Velocity': (omega3_sens, 2, 'Angular Velocity [rad/s]')
}

for idx, (title, (sensor, comp, ylabel)) in enumerate(sensors.items()):
    data = mbs.GetSensorStoredData(sensor)
    time = data[:, 0]
    values = data[:, 1 + comp]

    row = idx // 2
    col = idx % 2

    axs[row, col].plot(time, values, 'b-', linewidth=1.5)
    axs[row, col].set_title(title)
    axs[row, col].set_xlabel('Time [s]')
    axs[row, col].set_ylabel(ylabel)
    axs[row, col].grid(True)

plt.savefig('../solution/all_sensors.png', dpi=150)
plt.show()

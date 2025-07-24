import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
from helpful.constants import *

q0 = [0,0,0]
q1 = [0.2,-2*np.pi/3,-3*np.pi/4]

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

# Markers (used for constraints, not for setting node parameters)
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
zpos_sens = mbs.AddSensor(SensorBody(bodyNumber=b0, localPosition=joint0_pos, fileName='../solution/sensorPos0.txt',
                         outputVariableType=exu.OutputVariableType.Position))
zvel_sens = mbs.AddSensor(SensorBody(bodyNumber=b0, localPosition=joint0_pos, fileName='../solution/sensorVelPos0.txt',
                         outputVariableType=exu.OutputVariableType.Velocity))

theta1_sens = mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=joint1_pos, fileName='../solution/sensorPos1.txt',
                         outputVariableType=exu.OutputVariableType.Rotation))
omega1_sens = mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=joint1_pos, fileName='../solution/sensorVel1.txt',
                         outputVariableType=exu.OutputVariableType.AngularVelocity))

theta2_sens = mbs.AddSensor(SensorBody(bodyNumber=b2, localPosition=joint2_pos, fileName='../solution/sensorPos2.txt',
                         outputVariableType=exu.OutputVariableType.Rotation))
omega2_sens = mbs.AddSensor(SensorBody(bodyNumber=b2, localPosition=joint2_pos, fileName='../solution/sensorVel2.txt',
                         outputVariableType=exu.OutputVariableType.AngularVelocity))

theta3_sens = mbs.AddSensor(SensorBody(bodyNumber=b3, localPosition=joint3_pos, fileName='../solution/sensorPos3.txt',
                         outputVariableType=exu.OutputVariableType.Rotation))
omega3_sens = mbs.AddSensor(SensorBody(bodyNumber=b3, localPosition=joint3_pos, fileName='../solution/sensorVel3.txt',
                         outputVariableType=exu.OutputVariableType.AngularVelocity))



trajectory = Trajectory(initialCoordinates=q0, initialTime=0)

Kp_prismatic = 10000  # Proportional gain (Н/м)
Kd_prismatic = 1000   # Differential gain (Н·с/м)
Kp_revolute1 = 1000   # Proportional torque gain (Н·м/рад)
Kd_revolute1 = 100    # Differential torque gain (Н·м·с/рад)
Kp_revolute2 = 500
Kd_revolute2 = 50
Kp_revolute3 = 50
Kd_revolute3 = 10

markerBody0_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0, 0, 0]))
markerBody1_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0, 0, 0]))
markerBody2_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[0, 0, 0]))
markerBody3_com = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b3, localPosition=[0, 0, 0]))


def SmoothStep(x, x0, x1, value0, value1):
    if x <= x0:
        return value0
    elif x >= x1:
        return value1
    else:
        t = (x - x0) / (x1 - x0)
        return value0 + (value1 - value0) * (3*t**2 - 2*t**3)


def SmoothStepDerivative(x, x0, x1, value0, value1):
    loadValue = 0
    if x > x0 and x < x1:
        dx = x1-x0
        loadValue = (value1-value0) * 0.5*(pi/dx*sin((x-x0)/dx*pi))

    return loadValue


def d_desired(t):
    """Desired prismatic joint displacement"""
    return q1[0] * SmoothStep(t, 0, 1, 0, 1)


def d_desired_vel(t):
    """Desired prismatic joint velocity"""
    return q1[0] * SmoothStepDerivative(t, 0, 1, 0, 1)


def theta1_desired(t):
    """Desired Revolute1 angle"""
    return q1[1] * SmoothStep(t, 0, 1, 0, 1)


def theta1_desired_vel(t):
    """Desired Revolute1 angular velocity"""
    return q1[1] * SmoothStepDerivative(t, 0, 1, 0, 1)


def theta2_desired(t):
    """Desired Revolute2 angle"""
    return (q1[2] + q1[1]) * SmoothStep(t, 0, 1, 0, 1)


def theta2_desired_vel(t):
    """Desired Revolute2 angular velocity"""
    return (q1[2] + q1[1]) * SmoothStepDerivative(t, 0, 1, 0, 1)


def theta3_desired(t):
    """Desired Revolute2 angle"""
    return (-q1[2]/2 + q1[2] + q1[1]) * SmoothStep(t, 0, 1, 0, 1)


def theta3_desired_vel(t):
    """Desired Revolute2 angular velocity"""
    return (-q1[2]/2 + q1[2] + q1[1]) * SmoothStepDerivative(t, 0, 1, 0, 1)


def ForceControlZ(mbs, t, loadVector):
    """Prismatic joint force controller (ось Z)"""
    pos = mbs.GetNodeOutput(n0, exu.OutputVariableType.Position)
    vel = mbs.GetNodeOutput(n0, exu.OutputVariableType.Velocity)

    d_initial = com_cil_global[2]  # Initial Z displacement
    d_curr = pos[2] - d_initial  # Current displacement
    d_vel = vel[2]  # Current velocity

    # Desired values
    d_des = d_desired(t)
    d_des_v = d_desired_vel(t)

    # PD - controller
    F = Kp_prismatic * (d_des - d_curr) + Kd_prismatic * (d_des_v - d_vel)
    return [0, 0, F]  # Z-Force


def TorqueControlRevolute1(mbs, t, loadVector):
    """Toraue controller Revolute1 (ось Z)"""
    rot = mbs.GetNodeOutput(n1, exu.OutputVariableType.Rotation)
    theta_curr = rot[2] if isinstance(rot, (list, np.ndarray)) else 0
    omega_curr = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)[2]

    theta_des = theta1_desired(t)
    theta_des_v = theta1_desired_vel(t)

    T = Kp_revolute1 * (theta_des - theta_curr) + Kd_revolute1 * (theta_des_v - omega_curr)
    return [0, 0, T]


def TorqueControlRevolute2(mbs, t, loadVector):
    """Toraue controller Revolute2 (ось Z)"""
    rot = mbs.GetNodeOutput(n2, exu.OutputVariableType.Rotation)
    theta_curr = rot[2] if isinstance(rot, (list, np.ndarray)) else 0
    omega_curr = mbs.GetNodeOutput(n2, exu.OutputVariableType.AngularVelocity)[2]

    theta_des = theta2_desired(t)
    theta_des_v = theta2_desired_vel(t)

    T = Kp_revolute2 * (theta_des - theta_curr) + Kd_revolute2 * (theta_des_v - omega_curr)
    return [0, 0, T]


def TorqueControlRevolute3(mbs, t, loadVector):
    """Toraue controller Revolute1 (ось Z)"""
    rot = mbs.GetNodeOutput(n3, exu.OutputVariableType.Rotation)
    theta_curr = rot[2] if isinstance(rot, (list, np.ndarray)) else 0
    omega_curr = mbs.GetNodeOutput(n3, exu.OutputVariableType.AngularVelocity)[2]

    theta_des = theta3_desired(t)
    theta_des_v = theta3_desired_vel(t)

    T = Kp_revolute3 * (theta_des - theta_curr) + Kd_revolute3 * (theta_des_v - omega_curr)
    return [0, 0, T]



loadForceZ = mbs.AddLoad(LoadForceVector(
    markerNumber=markerBody0_com,
    loadVector=[0, 0, 0],
    bodyFixed=False,
    loadVectorUserFunction=ForceControlZ
))

loadTorque1 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=markerBody1_com,
    loadVector=[0, 0, 0],
    bodyFixed=True,
    loadVectorUserFunction=TorqueControlRevolute1
))

loadTorque2 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=markerBody2_com,
    loadVector=[0, 0, 0],
    bodyFixed=True,
    loadVectorUserFunction=TorqueControlRevolute2
))

loadTorque3 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=markerBody3_com,
    loadVector=[0, 0, 0],
    bodyFixed=True,
    loadVectorUserFunction=TorqueControlRevolute3
))

# Assemble and simulate
mbs.Assemble()
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])
SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()
mbs.SolutionViewer()
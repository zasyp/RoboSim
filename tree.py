import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP

# Initialize SystemContainer and MainSystem
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Constants (gravity, link lengths, masses)
g = [0, -9.81, 0]
l1 = 0.205
l2 = 0.205
l3 = 0.39
m1 = 6.936
m2 = 7.609
m3 = 0.533
m_cil = 9.838
w = 0.1

# Positions of links
com_cil_global = np.array([0, 0, -300/1000])
com1_global = np.array([0, 0, 0])
com2_global = np.array([-205/1000, 0, 81/1000])
com3_global = np.array([-410/1000, 0, 155/1000])

joint0_pos = np.array([0, 0, 0])
joint1_pos = np.array([0, 0, 300/1000])
joint2_pos = np.array([-l1, 0, com2_global[2]])
joint3_pos = np.array([-(l1 + l2), 0, com3_global[2]])

# Local center of mass positions
com2_local = com2_global - joint1_pos
com3_local = com3_global - joint2_pos

# Inertia tensors
inertiaTensor1 = np.array([
    [25749603.700/1e9, 25062.125/1e9, 9651359.305/1e9],
    [25062.125/1e9, 92340828.791/1e9, 24778.147/1e9],
    [9651359.305/1e9, 24778.147/1e9, 83172050.999/1e9]
])
inertiaTensor2 = np.array([
    [13449632.937/1e9, -24026.329/1e9, 1660746.774/1e9],
    [-24026.329/1e9, 84912103.395/1e9, 27847.617/1e9],
    [1660746.774/1e9, 27847.617/1e9, 89804961.922/1e9]
])
inertiaTensor3 = np.array([
    [480197.752/1e9, 835.157/1e9, 2908.542/1e9],
    [835.157/1e9, 14103107.261/1e9, 0/1e9],
    [2908.542/1e9, 0/1e9, 14577958.019/1e9]
])
inertiaTensorCilinder = np.array([
    [160256927.799232/1e9, -3211.288581/1e9, -57566.712025/1e9],
    [-3211.288581/1e9, 160272225.594173/1e9, -20361.408526/1e9],
    [-57566.712025/1e9, -20361.408526/1e9, 19416649.539239/1e9]
])

# Graphics from STL files
box_graphics = graphics.FromSTLfile('solution/box.stl', color=graphics.color.blue, scale=0.001)
graphicsBody1 = graphics.FromSTLfile('solution/link1.stl', color=graphics.color.dodgerblue, scale=0.001)
graphicsBody2 = graphics.FromSTLfile('solution/link2.stl', color=graphics.color.red, scale=0.001)
graphicsBody3 = graphics.FromSTLfile('solution/link3.stl', color=graphics.color.dodgerblue, scale=0.001)
graphicsBodyCilinder = graphics.FromSTLfile('solution/cilinder.stl', color=graphics.color.dodgerblue, scale=0.001)

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

# Markers for constraints and joints
markerPrismatic = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))  # Z-position for prismatic joint
markerRevolute1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n1, coordinate=6))  # Rotation Z for revolute joint 1
markerRevolute2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n2, coordinate=6))  # Rotation Z for revolute joint 2
markerRevolute3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n3, coordinate=6))  # Rotation Z for revolute joint 3
markerGroundCoord = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.AddNode(NodePointGround()), coordinate=0))  # Ground coordinate marker
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=joint0_pos))

# Joints
jointPrismatic = mbs.CreatePrismaticJoint(
    bodyNumbers=[oGround, b0],
    position=joint0_pos,
    useGlobalFrame=True,
    axis=[0, 0, 1],
    axisRadius=0.2*w,
    axisLength=300/1000
)
jointRevolute1 = mbs.CreateRevoluteJoint(
    bodyNumbers=[b0, b1],
    position=joint1_pos,
    axis=[0, 0, 1],
    axisRadius=0.2*w,
    axisLength=1*w
)
jointRevolute2 = mbs.CreateRevoluteJoint(
    bodyNumbers=[b1, b2],
    position=joint2_pos,
    axis=[0, 0, 1],
    axisRadius=0.2*w,
    axisLength=1*w
)
jointRevolute3 = mbs.CreateRevoluteJoint(
    bodyNumbers=[b2, b3],
    position=joint3_pos,
    axis=[0, 0, 1],
    axisRadius=0.2*w,
    axisLength=0.3*w
)

# Constraints
constraintPrismatic = mbs.AddObject(CoordinateConstraint(markerNumbers=[markerPrismatic, markerGroundCoord], offset=0))
constraintRevolute1 = mbs.AddObject(CoordinateConstraint(markerNumbers=[markerRevolute1, markerGroundCoord], offset=0))
constraintRevolute2 = mbs.AddObject(CoordinateConstraint(markerNumbers=[markerRevolute2, markerGroundCoord], offset=0))
theta_constraints23 = mbs.AddObject(CoordinateConstraint(markerNumbers=[markerRevolute3, markerRevolute2], factorValue1=-0.5))

# Sensors
mbs.AddSensor(SensorBody(
    bodyNumber=b0,
    localPosition=joint0_pos,
    fileName='solution/sensorPos0.txt',
    outputVariableType=exu.OutputVariableType.Position
))
mbs.AddSensor(SensorBody(
    bodyNumber=b1,
    localPosition=joint1_pos,
    fileName='solution/sensorPos1.txt',
    outputVariableType=exu.OutputVariableType.Rotation
))
mbs.AddSensor(SensorBody(
    bodyNumber=b2,
    localPosition=joint2_pos,
    fileName='solution/sensorPos2.txt',
    outputVariableType=exu.OutputVariableType.Rotation
))
mbs.AddSensor(SensorBody(
    bodyNumber=b3,
    localPosition=joint3_pos,
    fileName='solution/sensorPos3.txt',
    outputVariableType=exu.OutputVariableType.Rotation
))

# Trajectory
q0 = [0, 0, 0]  # Initial [d1, theta1, theta2]
q1 = [0.1, np.pi/2, np.pi/4]  # Final
trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
trajectory.Add(ProfileConstantAcceleration(q1, duration=1))
trajectory.Add(ProfileConstantAcceleration(q1, duration=2))

# PreStep user function
def PreStepUF(mbs, t):
    [u, v, a] = trajectory.Evaluate(t)  # u = [d1, theta1, theta2]
    mbs.SetObjectParameter(constraintPrismatic, 'offset', u[0])
    mbs.SetObjectParameter(constraintRevolute1, 'offset', u[1])
    mbs.SetObjectParameter(constraintRevolute2, 'offset', u[2])
    return True

mbs.SetPreStepUserFunction(PreStepUF)

# Simulation settings
tEnd = 3
h = 1e-4
simulationSettings = exu.SimulationSettings()
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
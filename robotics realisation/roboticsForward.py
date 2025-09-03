import os
import numpy as np
import matplotlib.pyplot as plt
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
from exudyn.robotics.special import *
from constants import *
from src.trajectory_opt import theta_from_time, veloc_from_time, tt
from tests import q_of_t, qd_of_t, qdd_of_t, t_max

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
q0 = np.array([0, 2*pi/3, -4*pi/3, 0])  # Initial configuration
q0 = [0, 0, 0, 0]
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
    preHT=HT_1,
    visualization=visualisationLink1,
    PDcontrol=(kp_rot, kd_rot)
))

robot.AddLink(RobotLink(
    mass=m2,
    COM=joint2_pos,
    inertia=inertiaTensor2,
    jointType='Rz',
    parent=1,  # Child of link 1
    preHT=HT_2,
    visualization=visualisationLink2,
    PDcontrol=(kp_rot2, kd_rot2)
))

robot.AddLink(RobotLink(
    mass=m3,
    COM=joint3_pos,
    inertia=inertiaTensor3,
    jointType='Rz',
    parent=2,  # Child of link 2
    preHT=HT_3,
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
# robotTrajectory = Trajectory(initialCoordinates=q0, initialTime=2)

# Define waypoints for the trajectory
q1 = [0.1, -0.5 * pi, 0.3 * pi, 0]
q2 = [0.2, 0.5 * pi, -0.3 * pi, 0]
q3 = [0.1, -0.5 * pi, -0.1 * pi, 0]
q4 = [0.3, -0.3 * pi, -0.4 * pi, 0]
q5 = q0

# Add motion profiles with constant acceleration
# robotTrajectory.Add(ProfileConstantAcceleration(q1, 1))
# robotTrajectory.Add(ProfileConstantAcceleration(q2, 1))
# robotTrajectory.Add(ProfileConstantAcceleration(q3, 1))
# robotTrajectory.Add(ProfileConstantAcceleration(q4, 1))
# robotTrajectory.Add(ProfileConstantAcceleration(q5, 1))
# ==========================================
# PRE-STEP USER FUNCTION FOR DYNAMIC CONTROL
# ==========================================
torque_values = []
def PreStepUF(mbs, t):
    if useKT:
        # saturate time to the defined profile
        t_clamped = t if t <= t_max else t_max

        q  = q_of_t(t_clamped)
        qd = qd_of_t(t_clamped)
        qdd= qdd_of_t(t_clamped)

        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector',  q.tolist())
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector',  qd.tolist())

        HT  = robot.JointHT(q)
        J   = JointJacobian(robot, HT, HT)
        M   = MassMatrix(robot, HT, J)
        tau_ff = (M @ qdd).tolist()
        mbs.SetObjectParameter(oKT, 'jointForceVector', tau_ff)

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
tEnd = 10  # Simulation time [s]
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

# Cleanup
exu.StopRenderer()
mbs.SolutionViewer()

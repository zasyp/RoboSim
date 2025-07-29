import numpy as np
import exudyn as exu
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfilePTP
from helpful.constants import *
from exudyn.itemInterface import LoadCoordinate

# Initialize system container and main system
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Add ground object with visualization
oGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
            visualization=VObjectGround(graphicsData=[graphics.Basis(0.04)])))

# Create base marker on ground
baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

# Initialize robot with gravity and base
robot = Robot(gravity=g, base=RobotBase())

# Define link visualizations
visualisationBox = VRobotLink(graphicsData=[graphicsBodyBox])
visualisationCylinder = VRobotLink(graphicsData=[graphicsBodyCylinder])
visualisationLink1 = VRobotLink(graphicsData=[graphicsBody1])
visualisationLink2 = VRobotLink(graphicsData=[graphicsBody2])
visualisationLink3 = VRobotLink(graphicsData=[graphicsBody3])

# Define robot links
linkBox = RobotLink(
    mass=m_box,
    COM=[0, 0, 0],
    inertia=inertiaTensorBox,
    parent=-1,
    visualization=visualisationBox,
    PDcontrol=(0, 0)
)
linkCylinder = RobotLink(
    mass=m_cyl,
    COM=com_cyl_global,
    inertia=inertiaTensorCilinder,
    jointType='Pz',
    parent=0,
    preHT=preHT_Cyl,
    visualization=visualisationCylinder,
    PDcontrol=(kp_trans, kd_trans)
)
link1 = RobotLink(
    mass=m1,
    COM=joint1_pos,
    inertia=inertiaTensor1,
    jointType='Rz',
    parent=1,
    preHT=preHT_1,
    visualization=visualisationLink1,
    PDcontrol=(kp_rot, kd_rot)
)
link2 = RobotLink(
    mass=m2,
    COM=joint2_pos,
    inertia=inertiaTensor2,
    jointType='Rz',
    parent=2,
    preHT=preHT_2,
    visualization=visualisationLink2,
    PDcontrol=(kp_rot, kd_rot)
)
link3 = RobotLink(
    mass=m3,
    COM=joint3_pos,
    inertia=inertiaTensor3,
    jointType='Rz',
    parent=3,
    preHT=preHT_3,
    visualization=visualisationLink3,
    PDcontrol=(0, 0)
)
# Add links to robot
robot.AddLink(linkBox)
robot.AddLink(linkCylinder)
robot.AddLink(link1)
robot.AddLink(link2)
robot.AddLink(link3)


tool = RobotTool(
    HT = HT_tool,
)
robot.tool = tool

# Create kinematic tree in multibody system
robotDict = robot.CreateKinematicTree(mbs=mbs)
oKT = robotDict['objectKinematicTree']

myIK = InverseKinematicsNumerical(robot, useRenderer=True)

if 1:
    T3 = np.array([
        [1, 0, 0, -0.205],
        [0, 1, 0, 0],
        [0, 0, 1, 0.074],
        [0, 0, 0, 1]
    ])

    q0 = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])

    sol = myIK.Solve(T3, q0=q0)
    print('success = {}\nq = {} rad'.format(sol[1], np.round(sol[0], 3)))

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

# Assemble system and start renderer
mbs.Assemble()
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])
SC.renderer.DoIdleTasks()


# Run dynamic simulation
mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)


SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()
mbs.SolutionViewer()
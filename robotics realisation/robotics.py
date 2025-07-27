import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.robotics import *
import numpy as np
from exudyn.kinematicTree import KinematicTree66, JointTransformMotionSubspace
from helpful.constants import *

# Initialize SystemContainer and MainSystem
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Add ground object
oGround = mbs.AddObject(ObjectGround(referencePosition=[0,0,0],
            visualization=VObjectGround(graphicsData=[graphics.Basis(0.5)])))

# Create base marker on ground
baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

# Initialize robot
robot = Robot(
    gravity=g,
    base=RobotBase()
)

# Define link visualizations
visualisationBox = VRobotLink(graphicsData=[graphicsBodyBox])
visualisationCylinder = VRobotLink(graphicsData=[graphicsBodyCylinder])
visualisationLink1 = VRobotLink(graphicsData=[graphicsBody1])
visualisationLink2 = VRobotLink(graphicsData=[graphicsBody2])
visualisationLink3 = VRobotLink(graphicsData=[graphicsBody3])

# Define links
linkBox = RobotLink(
    mass=m_box,
    COM=[0, 0, 0],
    inertia=inertiaTensorBox,
    parent=-1,
    visualization=visualisationBox,
)

linkCylinder = RobotLink(
    mass=m_cyl,
    COM=com_cyl_global,
    inertia=inertiaTensorCilinder,
    jointType='Pz',
    parent=0,
    preHT=preHT_Cyl,
    visualization=visualisationCylinder,
)

link1 = RobotLink(
    mass=m1,
    COM=joint1_pos,
    inertia=inertiaTensor1,
    jointType='Rz',
    parent=1,
    preHT=preHT_1,
    visualization=visualisationLink1
)

link2 = RobotLink(
    mass=m2,
    COM=joint2_pos,
    inertia=inertiaTensor2,
    jointType='Rz',
    parent=2,
    preHT=preHT_2,
    visualization=visualisationLink2
)

link3 = RobotLink(
    mass=m3,
    COM=joint3_pos,
    inertia=inertiaTensor3,
    jointType='Rz',
    parent=3,
    preHT=preHT_3,
    visualization=visualisationLink3
)



# Add links to robot
robot.AddLink(linkBox)
robot.AddLink(linkCylinder)
robot.AddLink(link1)
robot.AddLink(link2)
robot.AddLink(link3)

# Add robot to mbs using CreateRedundantCoordinateMBS
robotDict = robot.CreateRedundantCoordinateMBS(mbs, baseMarker=baseMarker, createJointTorqueLoads=False)

# Simulation settings
simulationSettings = exu.SimulationSettings()
tEnd = 3
h = 1e-3  # Increased for stability
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = False  # Disabled for stability
simulationSettings.solutionSettings.solutionWritePeriod = 0.005

# Visualization settings
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

# Assemble and run simulation
mbs.Assemble()
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])
SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()
mbs.SolutionViewer()
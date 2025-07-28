import numpy as np
import exudyn as exu
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfilePTP
from helpful.constants import *
from exudyn.itemInterface import LoadCoordinate

# Start and end joint coordinates
q0 = np.array([0, 0, 0])
q1 = np.array([0.2, np.pi/2, np.pi/3])

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

# Set PD control gains for joints that require control
kp_trans = 1e6  # Prismatic joint proportional gain [N/m]
kd_trans = 500  # Prismatic joint derivative gain [N·s/m]
kp_rot = 500    # Revolute joint proportional gain [Nm/rad]
kd_rot = 20     # Revolute joint derivative gain [Nm·s/rad]

linkCylinder.SetPDcontrol(kp_trans, kd_trans)
link1.SetPDcontrol(kp_rot, kd_rot)
link2.SetPDcontrol(kp_rot, kd_rot)

# Create kinematic tree in multibody system
robotDict = robot.CreateKinematicTree(mbs=mbs)
oKT = robotDict['objectKinematicTree']

# Create trajectory from q0 to q1
trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
trajectory.Add(ProfilePTP(q1, syncAccTimes=False, maxVelocities=[1, 1, 1], maxAccelerations=[2, 2, 2]))
trajectory.Initialize()

# Pre-step function to update joint target positions and velocities before each simulation step
def PreStepUF(mbs_, t):
    u, v, a = trajectory.Evaluate(t)
    mbs_.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
    mbs_.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
    return True

mbs.SetPreStepUserFunction(PreStepUF)

# Simulation settings
simulationSettings = exu.SimulationSettings()
tEnd = 30
h = 1e-3
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = False
simulationSettings.solutionSettings.solutionWritePeriod = 0.005

# Visualization settings
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = False

# Assemble system and start renderer
mbs.Assemble()
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])
SC.renderer.DoIdleTasks()

# Run dynamic simulation
mbs.SolveDynamic(
    simulationSettings=simulationSettings,
    solverType=exu.DynamicSolverType.TrapezoidalIndex2,
    showHints=True
)

# Wait for rendering to finish and cleanup
SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()

# Open solution viewer
mbs.SolutionViewer()

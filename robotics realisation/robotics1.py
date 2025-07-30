import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP
from exudyn.robotics.models import ManipulatorPuma560, ManipulatorPANDA, ManipulatorUR5, LinkDict2Robot
from exudyn.lieGroupBasics import LogSE3, ExpSE3
from helpful.constants import *

visualisationBox = VRobotLink(graphicsData=[graphicsBodyBox])
visualisationCylinder = VRobotLink(graphicsData=[graphicsBodyCylinder])
visualisationLink1 = VRobotLink(graphicsData=[graphicsBody1])
visualisationLink2 = VRobotLink(graphicsData=[graphicsBody2])
visualisationLink3 = VRobotLink(graphicsData=[graphicsBody3])


roboBase = RobotBase()
robot=Robot(gravity=g,
            base=roboBase,
            tool=RobotTool(HT=HT_tool))

robot.AddLink(robotLink=RobotLink(
    mass=m_box,
    COM=[0, 0, 0],
    inertia=inertiaTensorBox,
    parent=-1,
    visualization=visualisationBox,
    PDcontrol=(0, 0)
))

robot.AddLink(robotLink=RobotLink(
    mass=m_cyl,
    COM=com_cyl_global,
    inertia=inertiaTensorCilinder,
    jointType='Pz',
    parent=0,
    preHT=preHT_Cyl,
    visualization=visualisationCylinder,
    PDcontrol=(kp_trans, kd_trans)
))

robot.AddLink(robotLink=RobotLink(
    mass=m1,
    COM=joint1_pos,
    inertia=inertiaTensor1,
    jointType='Rz',
    parent=1,
    preHT=preHT_1,
    visualization=visualisationLink1,
    PDcontrol=(kp_rot, kd_rot)
))
robot.AddLink(robotLink=RobotLink(
    mass=m2,
    COM=joint2_pos,
    inertia=inertiaTensor2,
    jointType='Rz',
    parent=2,
    preHT=preHT_2,
    visualization=visualisationLink2,
    PDcontrol=(kp_rot, kd_rot)
))
robot.AddLink(robotLink=RobotLink(
    mass=m3,
    COM=joint3_pos,
    inertia=inertiaTensor3,
    jointType='Rz',
    parent=3,
    preHT=preHT_3,
    visualization=visualisationLink3,
    PDcontrol=(kp_rot, kd_rot)
))





SC = exu.SystemContainer()
mbs = SC.AddSystem()
robot.CreateKinematicTree(mbs=mbs)

q0 = np.array([0, 0, 0])
jointHTs = robot.JointHT(q0)

T = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0.1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
ik = InverseKinematicsNumerical(robot=robot, useRenderer=True,
                                flagDebug=True,
                                jointStiffness=1e1,
                                )
[q2, success] = ik.Solve(T, q0)

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

mbs.Assemble()
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])
SC.renderer.DoIdleTasks()


# Run dynamic simulation
mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.renderer.DoIdleTasks()
SC.renderer.Stop()
mbs.SolutionViewer()

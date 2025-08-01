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

useKT = True

robot=Robot(
    gravity=g,
    base=RobotBase(),
    tool=RobotTool(HT=HT_tool)
)

robot.AddLink(
    robotLink=RobotLink(
    mass=m_box,
    COM=[0, 0, 0],
    inertia=inertiaTensorBox,
    parent=-1,
    visualization=visualisationBox,
    PDcontrol=(0, 0)
))
robot.AddLink(
    robotLink=RobotLink(
    mass=m_cyl,
    COM=com_cyl_global,
    inertia=inertiaTensorCilinder,
    jointType='Pz',
    parent=0,
    preHT=preHT_Cyl,
    visualization=visualisationCylinder,
    PDcontrol=(kp_trans, kd_trans)
))
robot.AddLink(
    robotLink=RobotLink(
    mass=m1,
    COM=joint1_pos,
    inertia=inertiaTensor1,
    jointType='Rz',
    parent=1,
    preHT=preHT_1,
    visualization=visualisationLink1,
    PDcontrol=(kp_rot, kd_rot)
))
robot.AddLink(
    robotLink=RobotLink(
    mass=m2,
    COM=joint2_pos,
    inertia=inertiaTensor2,
    jointType='Rz',
    parent=2,
    preHT=preHT_2,
    visualization=visualisationLink2,
    PDcontrol=(kp_rot, kd_rot)
))
robot.AddLink(
    robotLink=RobotLink(
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
robotDict = robot.CreateKinematicTree(
    mbs=mbs,
    name="WHR"
)

oKT = robotDict['objectKinematicTree']
q0 = np.array([0, 0, 0, 0, 0])

robotTrajectory = Trajectory(initialCoordinates=q0, initialTime=0.25)
def PreStepUF(mbs, t):
    if useKT:
        [u,v,a] = robotTrajectory.Evaluate(t)
        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
    return True


mbs.SetPreStepUserFunction(PreStepUF)

q1 = [0, 0.1, -0.5 * pi, 0.3*pi, 0]  # zero angle configuration
q2 = [0, 0.2, -1.0 * pi, -0.3*pi, 0]  # zero angle configuration
q3 = [0, 0.3 , -0.75 * pi, 0.5*pi, 0]  # zero angle configuration
q4 = [0, 0.1 , -0.88 * pi, -0.3*pi, 0]  # zero angle configuration

robotTrajectory.Add(ProfileConstantAcceleration(q1,0.25))
robotTrajectory.Add(ProfileConstantAcceleration(q2,0.25))
robotTrajectory.Add(ProfileConstantAcceleration(q3,0.25))
robotTrajectory.Add(ProfileConstantAcceleration(q4,0.25))

mbs.Assemble()
simulationSettings = exu.SimulationSettings() #takes currently set values or default values
tEnd = 1.25 #simulation time
h = 0.25*1e-3 #step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms
SC.visualizationSettings.window.renderWindowSize=[1600,1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.showBasis=True
SC.visualizationSettings.general.drawWorldBasis=True
SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.openGL.lineWidth = 2
# SC.visualizationSettings.openGL.shadow=0.3 #don't do this for fine meshes!
SC.visualizationSettings.openGL.light0position=[-6,2,12,0]
exu.StartRenderer()
if 'renderState' in exu.sys: #reload old view
    SC.SetRenderState(exu.sys['renderState'])

mbs.WaitForUserToContinue() #stop before simulating
mbs.SolveDynamic(simulationSettings = simulationSettings,
                  solverType=exu.DynamicSolverType.TrapezoidalIndex2)
exu.StopRenderer() #safely close rendering window!

mbs.SolutionViewer()

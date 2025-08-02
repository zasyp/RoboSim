import os
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

visualisationBox = VRobotBase(graphicsData=[graphicsBodyBox])
visualisationCylinder = VRobotLink(graphicsData=[graphicsBodyCylinder])
visualisationLink1 = VRobotLink(graphicsData=[graphicsBody1])
visualisationLink2 = VRobotLink(graphicsData=[graphicsBody2])
visualisationLink3 = VRobotLink(graphicsData=[graphicsBody3])

useKT = True
q0 = np.array([0, 0, 0, 0])

baseBox = RobotBase(visualization=visualisationBox)

robot=Robot(
    gravity=g,
    base=baseBox,
    tool=RobotTool(HT=HT_tool)
)

robot.AddLink(
    robotLink=RobotLink(
    mass=m_cyl,
    COM=com_cyl_global,
    inertia=inertiaTensorCilinder,
    jointType='Pz',
    parent=-2,
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
    parent=-2,
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
    parent=-2,
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
    parent=-2,
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
nodeNumber = mbs.GetObject(oKT)['nodeNumber']

mJoint2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=2))
mJoint3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=3))

boxJoint = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=0))
factorValue1 = -0.5

mbs.AddObject(ObjectConnectorCoordinate(
    markerNumbers=[mJoint3, mJoint2],
    factorValue1=factorValue1,
))

jointList = [0]*robot.NumberOfLinks()

robotTrajectory = Trajectory(initialCoordinates=q0, initialTime=0.25)
def PreStepUF(mbs, t):
    if useKT:
        [u,v,a] = robotTrajectory.Evaluate(t)
        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
    return True


mbs.SetPreStepUserFunction(PreStepUF)

q1 = [0.1, -0.5 * pi, 0.3*pi, 0]
q2 = [0.1, 1* pi, 0.15*pi, 0]
q3 = [0.1, 0.5 * pi, 0.8*pi, 0]
q4 = [0.1, -0.5 * pi, 0.3*pi, 0]
q5 = [0, 0, 0, 0]

robotTrajectory.Add(ProfileConstantAcceleration(q1,0.3))
robotTrajectory.Add(ProfileConstantAcceleration(q2,0.3))
robotTrajectory.Add(ProfileConstantAcceleration(q3,0.3))
robotTrajectory.Add(ProfileConstantAcceleration(q4,0.3))
robotTrajectory.Add(ProfileConstantAcceleration(q5,0.3))

output_dir = "sensor_outputs"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)


verticalDispSens = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 0,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.Displacement,
        storeInternal = True,             # если нужен mbs.GetSensorStoredData()
        writeToFile = True,               # включает составление файла
        fileName = os.path.join(output_dir, "verticalDisp.txt"),
        name = "verticalDisp"
    )
)
theta1Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 1,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.Rotation,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "theta1_deg.txt"),
        name = "theta1_deg"
    )
)

theta2Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 2,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.Rotation,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "theta2_deg.txt"),
        name = "theta2_deg"
    )
)
theta3Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 3,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.Rotation,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "theta3_deg.txt"),
        name = "theta3_deg"
    )
)


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
tEnd = 6 #simulation time
h = 0.25*1e-3 #step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms
simulationSettings.solutionSettings.sensorsWritePeriod  = 0.005
SC.visualizationSettings.window.renderWindowSize=[1600,1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint=False
SC.visualizationSettings.nodes.showBasis=True
SC.visualizationSettings.general.drawWorldBasis=True
SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.openGL.lineWidth = 2
SC.visualizationSettings.openGL.light0position=[-6,2,12,0]
exu.StartRenderer()
if 'renderState' in exu.sys: #reload old view
    SC.SetRenderState(exu.sys['renderState'])

mbs.WaitForUserToContinue() #stop before simulating
mbs.SolveDynamic(simulationSettings = simulationSettings,
                  solverType=exu.DynamicSolverType.TrapezoidalIndex2)

mbs.PlotSensor(sensorNumbers=[0], components=2, xLabel='time (s)', yLabel='Z-axis displacement (m)')
mbs.PlotSensor(sensorNumbers=[1], components=2, factors=180/pi, xLabel='time (s)', yLabel='Theta 1 (deg)')
mbs.PlotSensor(sensorNumbers=[2], components=2, factors=180/pi, xLabel='time (s)', yLabel='Theta 2 (deg)')
mbs.PlotSensor(sensorNumbers=[3], components=2, factors=180/pi, xLabel='time (s)', yLabel='Theta 3 (deg)')

exu.StopRenderer() #safely close rendering window!

mbs.SolutionViewer()




import os
import matplotlib.pyplot as plt
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
    PDcontrol=(kp_rot2, kd_rot2)
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
    PDcontrol=(0, 0)
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

factorValue1 = -0.5

mbs.AddObject(ObjectConnectorCoordinate(
    markerNumbers=[mJoint3, mJoint2],
    factorValue1=factorValue1,
))

jointList = [0]*robot.NumberOfLinks()

robotTrajectory = Trajectory(initialCoordinates=q0, initialTime=0)
def PreStepUF(mbs, t):
    if useKT:
        [u,v,a] = robotTrajectory.Evaluate(t)
        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
    return True

mbs.SetPreStepUserFunction(PreStepUF)

q1 = [0.1, -0.5 * pi, pi, 0]
q2 = [0.3, 1* pi, -0.15*pi, 0]
q3 = [0.2, 0.5 * pi, 0.8*pi, 0]
q4 = [0.15, -0.5 * pi, 0.3*pi, 0]
q5 = [0, 0, 0, 0]

robotTrajectory.Add(ProfileConstantAcceleration(q1,2))
robotTrajectory.Add(ProfileConstantAcceleration(q2,2))
robotTrajectory.Add(ProfileConstantAcceleration(q3,2))
robotTrajectory.Add(ProfileConstantAcceleration(q4,2))
robotTrajectory.Add(ProfileConstantAcceleration(q5,2))


output_dir = "sensor_outputs"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)


verticalDispSens = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 0,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.Displacement,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "verticalDisp.txt"),
        name = "verticalDisp"
    )
)
verticalVelSens = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 0,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.VelocityLocal,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "verticalVel.txt"),
        name = "verticalVel"
    )
)
verticalAccSens = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 0,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.AccelerationLocal,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "verticalAcc.txt"),
        name = "verticalAcc"
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
omega1Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 1,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.AngularVelocity,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "omega1_deg.txt"),
        name = "omega1_deg"
    )
)
epsilon1Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 1,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.AngularAccelerationLocal,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "epsilon1_deg.txt"),
        name = "epsilon1_deg"
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
omega2Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 2,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.AngularVelocity,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "omega2_deg.txt"),
        name = "omega2_deg"
    )
)
epsilon2Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 2,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.AngularAccelerationLocal,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "epsilon2_deg.txt"),
        name = "epsilon2_deg"
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
omega3Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 3,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.AngularVelocity,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "omega3_deg.txt"),
        name = "omega3_deg"
    )
)
epsilon3Sensor = mbs.AddSensor(
    SensorKinematicTree(
        objectNumber = oKT,
        linkNumber = 3,
        localPosition = [0,0,0],
        outputVariableType = exu.OutputVariableType.AngularAccelerationLocal,
        storeInternal = True,
        writeToFile = True,
        fileName = os.path.join(output_dir, "epsilon3_deg.txt"),
        name = "epsilon3_deg"
    )
)


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 30 #simulation time
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

# Data for building plots
verticalDisp_data = mbs.GetSensorStoredData(verticalDispSens)
theta1_data = mbs.GetSensorStoredData(theta1Sensor)
theta2_data = mbs.GetSensorStoredData(theta2Sensor)
theta3_data = mbs.GetSensorStoredData(theta3Sensor)

verticalVel_data = mbs.GetSensorStoredData(verticalVelSens)
omega1_data = mbs.GetSensorStoredData(omega1Sensor)
omega2_data = mbs.GetSensorStoredData(omega2Sensor)
omega3_data = mbs.GetSensorStoredData(omega3Sensor)

verticalAcc_data = mbs.GetSensorStoredData(verticalAccSens)
epsilon1_data = mbs.GetSensorStoredData(epsilon1Sensor)
epsilon2_data = mbs.GetSensorStoredData(epsilon2Sensor)
epsilon3_data = mbs.GetSensorStoredData(epsilon3Sensor)

# Get solution times and initialize ideal trajectory arrays
times = theta1_data[:, 0]
# Extract actual sensor data
verticalDisp = verticalDisp_data[:, 3]  # z-component
theta1 = theta1_data[:, 3]  # rotation z-component (in radians)
theta2 = theta2_data[:, 3]  # rotation z-component (in radians)
theta3 = theta3_data[:, 3]  # rotation z-component (in radians)

verticalVel = verticalVel_data[:, 3]  # z-velocity
omega1 = omega1_data[:, 3]  # angular velocity z-component (in rad/s)
omega2 = omega2_data[:, 3]  # angular velocity z-component (in rad/s)
omega3 = omega3_data[:, 3]  # angular velocity z-component (in rad/s)

# Calculate ideal trajectory values
n = len(times)
ideal_positions = np.zeros((n, 4))  # [vertical, theta1, theta2, theta3]
ideal_velocities = np.zeros((n, 4))

for i, t in enumerate(times):
    u, v, a = robotTrajectory.Evaluate(t)
    ideal_positions[i] = u
    ideal_velocities[i] = v

# Calculate initial offset for vertical position
z0 = verticalDisp_data[0, 3]  # initial z-position
ideal_vertical_position = z0 + ideal_positions[:, 0]

# ===========================
# MAIN PLOT (3x4 grid)
# ===========================
plt.figure(figsize=(20, 15))

# Position plots (row 1)
# Vertical position
plt.subplot(3, 4, 1)
plt.plot(times, ideal_vertical_position, 'b-', label='Ideal')
plt.plot(times, verticalDisp, 'r--', label='Actual')
plt.title('Vertical Position (m)')
plt.legend()
plt.grid()

# Theta1
plt.subplot(3, 4, 2)
plt.plot(times, ideal_positions[:, 1], 'b-', label='Ideal')
plt.plot(times, theta1, 'r--', label='Actual')
plt.title('Theta1 (rad)')
plt.legend()
plt.grid()

# Theta2
plt.subplot(3, 4, 3)
plt.plot(times, ideal_positions[:, 2], 'b-', label='Ideal')
plt.plot(times, theta2, 'r--', label='Actual')
plt.title('Theta2 (rad)')
plt.legend()
plt.grid()

# Theta3
plt.subplot(3, 4, 4)
plt.plot(times, theta3, 'r--', label='Actual')
plt.title('Theta3 (rad)')
plt.legend()
plt.grid()

# Velocity plots (row 2)
# Vertical velocity
plt.subplot(3, 4, 5)
plt.plot(times, ideal_velocities[:, 0], 'b-', label='Ideal')
plt.plot(times, verticalVel, 'r--', label='Actual')
plt.title('Vertical Velocity (m/s)')
plt.legend()
plt.grid()

# Omega1
plt.subplot(3, 4, 6)
plt.plot(times, ideal_velocities[:, 1], 'b-', label='Ideal')
plt.plot(times, omega1, 'r--', label='Actual')
plt.title('Omega1 (rad/s)')
plt.legend()
plt.grid()

# Omega2
plt.subplot(3, 4, 7)
plt.plot(times, ideal_velocities[:, 2], 'b-', label='Ideal')
plt.plot(times, omega2, 'r--', label='Actual')
plt.title('Omega2 (rad/s)')
plt.legend()
plt.grid()

# Omega3
plt.subplot(3, 4, 8)
plt.plot(times, omega3, 'r--', label='Actual')
plt.title('Omega3 (rad/s)')
plt.legend()
plt.grid()

# Acceleration plots (row 3) - unchanged
plt.subplot(3, 4, 9)
plt.plot(verticalAcc_data[:, 0], verticalAcc_data[:, 3])
plt.title('Vertical Acceleration (m/s²)')
plt.grid()

plt.subplot(3, 4, 10)
plt.plot(epsilon1_data[:, 0], epsilon1_data[:, 3])
plt.title('Epsilon1 (rad/s²)')
plt.grid()

plt.subplot(3, 4, 11)
plt.plot(epsilon2_data[:, 0], epsilon2_data[:, 3])
plt.title('Epsilon2 (rad/s²)')
plt.grid()

plt.subplot(3, 4, 12)
plt.plot(epsilon3_data[:, 0], epsilon3_data[:, 3])
plt.title('Epsilon3 (rad/s²)')
plt.grid()

plt.tight_layout(pad=2.0)
plt.savefig('all_sensors_data_with_ideal.png')
plt.close()

# =================================
# ERROR PLOTS (vertical, theta1, theta2)
# =================================
plt.figure(figsize=(12, 8))

# Vertical position error
error_vertical = ideal_vertical_position - verticalDisp
plt.subplot(3, 1, 1)
plt.plot(times, error_vertical)
plt.title('Vertical Position Error')
plt.ylabel('Error (m)')
plt.grid()

# Theta1 error (in radians)
error_theta1 = ideal_positions[:, 1] - theta1
plt.subplot(3, 1, 2)
plt.plot(times, error_theta1)
plt.title('Theta1 Error')
plt.ylabel('Error (rad)')
plt.grid()

# Theta2 error (in radians)
error_theta2 = ideal_positions[:, 2] - theta2
plt.subplot(3, 1, 3)
plt.plot(times, error_theta2)
plt.title('Theta2 Error')
plt.ylabel('Error (rad)')
plt.xlabel('Time (s)')
plt.grid()

plt.tight_layout()
plt.savefig('position_errors.png')
plt.close()

exu.StopRenderer()
mbs.SolutionViewer()
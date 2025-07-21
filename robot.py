import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Константы (УСП, длины звеньев, массы подвижных звеньев)
g = [0, -9.81, 0]

l1 = 0.205
l2 = 0.205
l3 = 0.39

m1 = 6.936
m2 = 7.609
m3 = 0.533
m_cil = 9.838

w = 0.1

# Положения звеньев
com_cil_global = np.array([0,0,-300/1000])
com1_global = np.array([0,0,0])
com2_global = np.array([-205/1000, 0, 81 / 1000])
com3_global = np.array([-410 / 1000, 0 / 1000, 155 / 1000])


joint0_pos = np.array([0, 0, 0])
joint1_pos = np.array([0, 0, 300])
joint2_pos = np.array([-l1, 0, com2_global[2]])
joint3_pos = np.array([-(l1 + l2), 0, com3_global[2]])


# Расчет положений звеньев в локальных координатах
com2_local = com2_global - joint1_pos
com3_local = com3_global - joint2_pos

# Тензоры инерции
inertiaTensor1 = np.array([
    [25749603.700 / 1e9, 25062.125 / 1e9, 9651359.305 / 1e9],
    [25062.125 / 1e9, 92340828.791 / 1e9, 24778.147 / 1e9],
    [9651359.305 / 1e9, 24778.147 / 1e9, 83172050.999 / 1e9]
])

inertiaTensor2 = np.array([
    [13449632.937 / 1e9, -24026.329 / 1e9, 1660746.774 / 1e9],
    [-24026.329 / 1e9, 84912103.395 / 1e9, 27847.617 / 1e9],
    [1660746.774 / 1e9, 27847.617 / 1e9, 89804961.922 / 1e9]
])

inertiaTensor3 = np.array([
    [480197.752 / 1e9, 835.157 / 1e9, 2908.542 / 1e9],
    [835.157 / 1e9, 14103107.261 / 1e9, 0 / 1e9],
    [2908.542 / 1e9, 0 / 1e9, 14577958.019 / 1e9]
])

inertiaTensorCilinder = np.array([
    [160256927.799232 / 1e9, -3211.288581 / 1e9, -57566.712025 / 1e9],
    [-3211.288581 / 1e9, 160272225.594173 / 1e9, -20361.408526 / 1e9],
    [-57566.712025 / 1e9, -20361.408526 / 1e9, 19416649.539239 / 1e9]
])

# Импорт графики из файлов
box_graphics = graphics.FromSTLfile('solution/box.stl',
                                     color=graphics.color.blue,
                                     scale=0.001)
graphicsBody1 = graphics.FromSTLfile('solution/link1.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)
graphicsBody2 = graphics.FromSTLfile('solution/link2.stl',
                                     color=graphics.color.red,
                                     scale=0.001)
graphicsBody3 = graphics.FromSTLfile('solution/link3.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)

graphicsBodyCilinder = graphics.FromSTLfile('solution/cilinder.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)

oGround = mbs.CreateGround(referencePosition=[0,-145/1000,-805/1000], graphicsDataList=[box_graphics])

# Тензоры инерции подвижных звеньев
i1 = RigidBodyInertia(mass=m1, inertiaTensor=inertiaTensor1)
i2 = RigidBodyInertia(mass=m2, inertiaTensor=inertiaTensor2)
i3 = RigidBodyInertia(mass=m3, inertiaTensor=inertiaTensor3)
iCilinder = RigidBodyInertia(mass=m_cil, inertiaTensor=inertiaTensorCilinder)

# Ввод звеньев ([номер node, номер body])
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

link1_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate=6))
link2_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n2, coordinate=6))
link3_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n3, coordinate=6))

markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=joint0_pos))

# joint'ы
jointPrismatic = mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=joint0_pos,
                         useGlobalFrame=True, axis=[0,0,0.3],
                         axisRadius=0.2*w, axisLength=300/1000,
                         )

jointRevolute1 = mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], position=joint1_pos,
                        axis=[0,0,1], axisRadius=0.2*w, axisLength=1*w)

jointRevolute2 = mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=joint2_pos,
                        axis=[0,0,1], axisRadius=0.2*w, axisLength=1*w)

jointRevolute3 = mbs.CreateRevoluteJoint(bodyNumbers=[b2, b3], position=joint3_pos,
                        axis=[0,0,1], axisRadius=0.2*w, axisLength=0.3*w)

simulationSettings = exu.SimulationSettings()

# Моменты и силы
torque = [0,0,0.1]
torque2 = [0,0,-0.05]

force = [0,0.05,0]
vert_force = [0,0,0.5]

# mbs.CreateForce(bodyNumber=b4,
#                 loadVector=vert_force,
#                 localPosition=[0, 0, 0],
#                 bodyFixed=False)

mbs.CreateTorque(bodyNumber=b2,
                loadVector=torque,
                localPosition=[0,0,0],
                bodyFixed=False)

# mbs.CreateTorque(bodyNumber=b3,
#                 loadVector=torque,
#                 localPosition=[0,0,0],
#                 bodyFixed=True)

mbs.CreateTorque(bodyNumber=b1,
                loadVector=torque2,
                localPosition=[0,0,0],
                bodyFixed=True)

# Сенсоры
joint1_sens=mbs.AddSensor(SensorBody(bodyNumber = b1, localPosition = joint1_pos,
                               fileName = 'solution/sensorPos.txt',
                               outputVariableType = exu.OutputVariableType.AngularAcceleration))

joint2_sens=mbs.AddSensor(SensorBody(bodyNumber = b2, localPosition = joint2_pos,
                               fileName = 'solution/sensorPos.txt',
                               outputVariableType = exu.OutputVariableType.AngularAcceleration))

joint3_sens=mbs.AddSensor(SensorBody(bodyNumber = b3, localPosition = joint3_pos,
                               fileName = 'solution/sensorPos.txt',
                               outputVariableType = exu.OutputVariableType.AngularAcceleration))

# Constraint'ы
theta_constraints23 = mbs.AddObject(CoordinateConstraint(markerNumbers=[link3_marker, link2_marker],
                                                       factorValue1=-0.5, ))


mbs.Assemble()

tEnd = 3
h = 1e-4
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005

SC.visualizationSettings.window.renderWindowSize = [1600,1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False

SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

exu.StartRenderer()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])

mbs.WaitForUserToContinue()

mbs.SolveDynamic(simulationSettings = simulationSettings,
                 solverType = exu.DynamicSolverType.TrapezoidalIndex2)

mbs.SolveDynamic(simulationSettings = simulationSettings)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()
mbs.SolutionViewer()
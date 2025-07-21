import exudyn as exu
from exudyn.utilities import *
from exudyn.robotics import *
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

# Системный контейнер
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Параметры гравитации
g = [0, -9.81, 0]

# Параметры звеньев
l1 = 0.205  # длина звена 1
l2 = 0.205  # длина звена 2
l3 = 0.39   # длина звена 3

# Массы звеньев
m_cil = 9.838  # масса цилиндра (b0)
m1 = 6.936     # масса звена b1
m2 = 7.609     # масса звена b2
m3 = 0.533     # масса звена b3

# Тензоры инерции (примерные значения из исходного кода)
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

# Графика для звеньев (заглушка, замените на свои STL-файлы)
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

# Словарь ссылок с параметрами Денавита-Хартенберга
linkDict = {
    0: {
        'jointType': 'prismatic',
        'stdDH': [0, 0, 'variable', 0],  # alpha, a, d, theta
        'mass': m_cil,
        'inertia': inertiaTensorCilinder,
        'COM': [0, 0, -0.3],
        'graphics': [graphicsBodyCilinder]
    },
    1: {
        'jointType': 'revolute',
        'stdDH': [0, -l1, 0.3, 'variable'],
        'mass': m1,
        'inertia': inertiaTensor1,
        'COM': [0, 0, -0.3],
        'graphics': [graphicsBody1]
    },
    2: {
        'jointType': 'revolute',
        'stdDH': [0, -l2, -0.219, 'variable'],
        'mass': m2,
        'inertia': inertiaTensor2,
        'COM': [0, 0, 0],
        'graphics': [graphicsBody2]
    },
    3: {
        'jointType': 'revolute',
        'stdDH': [0, -l3, 0.074, 'variable'],
        'mass': m3,
        'inertia': inertiaTensor3,
        'COM': [0, 0, 0],
        'graphics': [graphicsBody3]
    }
}
# Создание объекта робота
robot = Robot(
    gravity=g,
    base=RobotBase(HT=HTtranslate([0, -0.145, -0.805]), visualization=VRobotBase(graphicsData=[box_graphics])),
    tool=RobotTool(HT=HTtranslate([0, 0, 0]), visualization=VRobotTool(graphicsData=[])),
    referenceConfiguration=[0, 0, 0, 0]  # начальные значения [d1, theta1, theta2, theta3]
)

# Добавление робота в систему
robot = LinkDict2Robot(linkDict, robot)

# Создание кинематического дерева
robotDict = robot.CreateKinematicTree(mbs)
oKT = robotDict['objectKinematicTree']

# Получение номеров узлов для суставов
nodeGeneric = robotDict['nodeGeneric']

# Добавление маркеров для ограничения φ₃ = -φ₂ / 2
link2_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeGeneric, coordinate=2))  # theta2
link3_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodeGeneric, coordinate=3))  # theta3

# Ограничение φ₃ = -φ₂ / 2
theta_constraints23 = mbs.AddObject(CoordinateConstraint(
    markerNumbers=[link3_marker, link2_marker],
    factorValue1=-0.5
))

# Сенсоры для записи углов
sJoint1 = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT, linkNumber=1, localPosition=HT2translation(joint1_pos),
    fileName='solution/sensorJoint1.txt', outputVariableType=exu.OutputVariableType.Rotation
))
sJoint2 = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT, linkNumber=2, localPosition=HT2translation(joint2_pos),
    fileName='solution/sensorJoint2.txt', outputVariableType=exu.OutputVariableType.Rotation
))
sJoint3 = mbs.AddSensor(SensorKinematicTree(
    objectNumber=oKT, linkNumber=3, localPosition=HT2translation(joint3_pos),
    fileName='solution/sensorJoint3.txt', outputVariableType=exu.OutputVariableType.Rotation
))

# Настройки симуляции
tEnd = 3
h = 1e-4
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
simulationSettings.timeIntegration.newton.useModifiedNewton = True

# Настройки визуализации
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.connectors.showJointAxes = True
SC.visualizationSettings.connectors.jointAxesLength = 0.02
SC.visualizationSettings.connectors.jointAxesRadius = 0.002

# Сборка системы
mbs.Assemble()

# Запуск рендера
exu.StartRenderer()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])

# Ожидание пользователя
mbs.WaitForUserToContinue()

# Выполнение симуляции
mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

# Остановка рендера и запуск SolutionViewer
SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()
mbs.SolutionViewer()
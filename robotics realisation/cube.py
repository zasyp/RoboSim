import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
import numpy as np

# Создаем систему
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Определяем гравитацию
g = [0, 0, -9.81]

# Создаем базовый объект робота
roboBase = RobotBase()

# Визуализация кубика
visualisationCube = VRobotLink(graphicsData=[graphics.Brick([0,0,0], [0.1,0.1,0.1], graphics.color.red)])

# Создаем робота с одним поступательным суставом (Pz)
robot = Robot(gravity=g, base=roboBase, tool=RobotTool(HT=HT0()))
robot.AddLink(robotLink=RobotLink(
    mass=1.0,
    COM=[0, 0, 0],
    inertia=np.diag([0.1, 0.1, 0.1]),
    jointType='Pz',
    parent=-1,
    visualization=visualisationCube,
    PDcontrol=(10, 10),
))

# Начальная конфигурация
q0 = np.array([0.0])  # начальное положение сустава

# Создаем объект для обратной кинематики
ik = InverseKinematicsNumerical(robot=robot, useRenderer=False, jointStiffness=1e4)

# Начальная поза
T_initial = robot.JointHT(q0)[-1] @ robot.tool.HT

# Конечная поза (движение на 0.1 м вдоль Z)
T_final = T_initial.copy()
T_final[2, 3] = 0.3  # устанавливаем Z=0.1 м




# Настройка симуляции
simulationSettings = exu.SimulationSettings()
tEnd = 3
h = 1e-4
simulationSettings.timeIntegration.numberOfSteps = int(tEnd / h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005

# Визуализация
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False

# Создаем кинематическое дерево
robotDict = robot.CreateKinematicTree(mbs=mbs)
oKT = robotDict['objectKinematicTree']

# Устанавливаем начальное положение
mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', q0)

# Вычисляем решение обратной кинематики
try:
    [q_final, success] = ik.SolveSafe(T_final, q0)
    if not success:
        print("Решение обратной кинематики не найдено. Используется начальная позиция.")
        q_final = q0  # используем начальное положение при ошибке
    else:
        print(f"Решение найдено: q_final = {q_final}")
except Exception as e:
    print(f"Ошибка обратной кинематики: {e}")
    q_final = q0  # используем начальное положение при ошибке

# Сборка и запуск симуляции
mbs.Assemble()
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])

mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.renderer.Stop()
mbs.SolutionViewer()
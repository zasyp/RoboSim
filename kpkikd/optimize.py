import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np
from exudyn.robotics.motion import Trajectory, ProfilePTP
from scipy.optimize import minimize
from helpful.constants import *

# Начальные и конечные координаты
q0 = [0, 0, 0, 0]
q1 = [0.2, -2*np.pi/3, -2*np.pi/3, -np.pi]

# Инициализация SystemContainer и MainSystem
SC = exu.SystemContainer()
mbs = SC.AddSystem()

# Основание и тела
oGround = mbs.CreateGround(referencePosition=[0, -145/1000, -805/1000], graphicsDataList=[box_graphics])

i1 = RigidBodyInertia(mass=m1, inertiaTensor=inertiaTensor1)
i2 = RigidBodyInertia(mass=m2, inertiaTensor=inertiaTensor2)
i3 = RigidBodyInertia(mass=m3, inertiaTensor=inertiaTensor3)
iCilinder = RigidBodyInertia(mass=m_cil, inertiaTensor=inertiaTensorCilinder)

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

# Маркеры
link0_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=2))
link1_marker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0, 0, 0]))
link2_marker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[0, 0, 0]))
link3_marker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b3, localPosition=[0, 0, 0]))

markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 0]))
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=joint0_pos))

# Соединения
jointPrismatic = mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=joint0_pos,
                                          useGlobalFrame=True, axis=[0, 0, 1],
                                          axisRadius=0.2*w, axisLength=300/1000)
jointRevolute1 = mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], position=joint1_pos,
                                         axis=[0, 0, 1], axisRadius=0.2*w, axisLength=1*w)
jointRevolute2 = mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=joint2_pos,
                                         axis=[0, 0, 1], axisRadius=0.2*w, axisLength=1*w)
jointRevolute3 = mbs.CreateRevoluteJoint(bodyNumbers=[b2, b3], position=joint3_pos,
                                         axis=[0, 0, 1], axisRadius=0.2*w, axisLength=0.3*w)

# Настройки симуляции
simulationSettings = exu.SimulationSettings()
tEnd = 3
h = 1e-4
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005

# Настройки визуализации
SC.visualizationSettings.window.renderWindowSize = [1600, 1200]
SC.visualizationSettings.openGL.multiSampling = 4
SC.visualizationSettings.general.autoFitScene = False
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True

# Траекторные профили
traj_d = Trajectory(initialCoordinates=[0], initialTime=0)
traj_d.Add(ProfilePTP([q1[0]], maxVelocities=[1.0], maxAccelerations=[2.0], syncAccTimes=False))

traj_theta1 = Trajectory(initialCoordinates=[0], initialTime=0)
traj_theta1.Add(ProfilePTP([q1[1]], maxVelocities=[10.0], maxAccelerations=[20.0], syncAccTimes=False))

traj_theta2 = Trajectory(initialCoordinates=[0], initialTime=0)
traj_theta2.Add(ProfilePTP([q1[2]], maxVelocities=[10.0], maxAccelerations=[20.0], syncAccTimes=False))

traj_theta3 = Trajectory(initialCoordinates=[q1[1]+q1[2]], initialTime=0)
traj_theta3.Add(ProfilePTP([-q1[2]/2], maxVelocities=[10.0], maxAccelerations=[20.0], syncAccTimes=False))

# Начальные коэффициенты PID
Kp_prismatic = 1000
Ki_prismatic = 1000
Kd_prismatic = 700
Kp_revolute1 = 500
Ki_revolute1 = 500
Kd_revolute1 = 600
Kp_revolute2 = 120
Ki_revolute2 = 1000
Kd_revolute2 = 200
Kp_revolute3 = 100
Ki_revolute3 = 500
Kd_revolute3 = 600

# Глобальные переменные для интегралов и стоимости
integral_d = 0
integral_theta1 = 0
integral_theta2 = 0
integral_theta3 = 0
cost_d = 0
cost_theta1 = 0
cost_theta2 = 0
cost_theta3 = 0
t_prev = 0

# Функции управления с отслеживанием стоимости
def ForceControlZ(mbs, t, loadVector):
    global integral_d, cost_d, t_prev
    positions, velocities, _ = traj_d.Evaluate(t)
    d_des = positions[0]
    d_des_v = velocities[0]
    pos = mbs.GetNodeOutput(n0, exu.OutputVariableType.Position)
    vel = mbs.GetNodeOutput(n0, exu.OutputVariableType.Velocity)
    d_initial = com_cil_global[2]
    d_curr = pos[2] - d_initial
    d_vel = vel[2]
    error = d_des - d_curr
    dt = t - t_prev
    if dt > 0:
        integral_d += error * dt
        cost_d += error**2 * dt
    t_prev = t
    F = Kp_prismatic * error + Ki_prismatic * integral_d + Kd_prismatic * (d_des_v - d_vel)
    return [0, 0, F]

def TorqueControlRevolute1(mbs, t, loadVector):
    global integral_theta1, cost_theta1, t_prev
    positions, velocities, _ = traj_theta1.Evaluate(t)
    theta_des = positions[0]
    theta_des_v = velocities[0]
    rot = mbs.GetNodeOutput(n1, exu.OutputVariableType.Rotation)
    theta_curr = rot[2]
    omega_curr = mbs.GetNodeOutput(n1, exu.OutputVariableType.AngularVelocity)[2]
    error = theta_des - theta_curr
    dt = t - t_prev
    if dt > 0:
        integral_theta1 += error * dt
        cost_theta1 += error**2 * dt
    t_prev = t
    T = Kp_revolute1 * error + Ki_revolute1 * integral_theta1 + Kd_revolute1 * (theta_des_v - omega_curr)
    return [0, 0, T]

def TorqueControlRevolute2(mbs, t, loadVector):
    global integral_theta2, cost_theta2, t_prev
    positions, velocities, _ = traj_theta2.Evaluate(t)
    theta_des = positions[0] + q1[1]
    theta_des_v = velocities[0]
    rot = mbs.GetNodeOutput(n2, exu.OutputVariableType.Rotation)
    theta_curr = rot[2]
    omega_curr = mbs.GetNodeOutput(n2, exu.OutputVariableType.AngularVelocity)[2]
    error = theta_des - theta_curr
    dt = t - t_prev
    if dt > 0:
        integral_theta2 += error * dt
        cost_theta2 += error**2 * dt
    t_prev = t
    T = Kp_revolute2 * error + Ki_revolute2 * integral_theta2 + Kd_revolute2 * (theta_des_v - omega_curr)
    return [0, 0, T]

def TorqueControlRevolute3(mbs, t, loadVector):
    global integral_theta3, cost_theta3, t_prev
    positions, velocities, _ = traj_theta3.Evaluate(t)
    theta_des = positions[0] + q1[1] + q1[2]
    theta_des_v = velocities[0]
    rot = mbs.GetNodeOutput(n3, exu.OutputVariableType.Rotation)
    theta_curr = rot[2]
    omega_curr = mbs.GetNodeOutput(n3, exu.OutputVariableType.AngularVelocity)[2]
    error = theta_des - theta_curr
    dt = t - t_prev
    if dt > 0:
        integral_theta3 += error * dt
        cost_theta3 += error**2 * dt
    t_prev = t
    T = Kp_revolute3 * error + Ki_revolute3 * integral_theta3 + Kd_revolute3 * (theta_des_v - omega_curr)
    return [0, 0, T]

# Добавление нагрузок
loadForceZ = mbs.AddLoad(LoadForceVector(
    markerNumber=markerBody0J0,
    loadVector=[0, 0, 0],
    bodyFixed=False,
    loadVectorUserFunction=ForceControlZ
))

loadTorque1 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=link1_marker,
    loadVector=[0, 0, 0],
    bodyFixed=True,
    loadVectorUserFunction=TorqueControlRevolute1
))

loadTorque2 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=link2_marker,
    loadVector=[0, 0, 0],
    bodyFixed=True,
    loadVectorUserFunction=TorqueControlRevolute2
))

loadTorque3 = mbs.AddLoad(LoadTorqueVector(
    markerNumber=link3_marker,
    loadVector=[0, 0, 0],
    bodyFixed=True,
    loadVectorUserFunction=TorqueControlRevolute3
))

# Сборка системы
mbs.Assemble()

# Функция симуляции для оптимизации
def run_simulation():
    mbs.Reset()
    mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

# Функции стоимости для оптимизации
def cost_function_prismatic(pid_params):
    global Kp_prismatic, Ki_prismatic, Kd_prismatic, cost_d, integral_d
    Kp_prismatic, Ki_prismatic, Kd_prismatic = [max(p, 0) for p in pid_params]
    cost_d = 0
    integral_d = 0
    run_simulation()
    return cost_d

def cost_function_revolute1(pid_params):
    global Kp_revolute1, Ki_revolute1, Kd_revolute1, cost_theta1, integral_theta1
    Kp_revolute1, Ki_revolute1, Kd_revolute1 = [max(p, 0) for p in pid_params]
    cost_theta1 = 0
    integral_theta1 = 0
    run_simulation()
    return cost_theta1

def cost_function_revolute2(pid_params):
    global Kp_revolute2, Ki_revolute2, Kd_revolute2, cost_theta2, integral_theta2
    Kp_revolute2, Ki_revolute2, Kd_revolute2 = [max(p, 0) for p in pid_params]
    cost_theta2 = 0
    integral_theta2 = 0
    run_simulation()
    return cost_theta2

def cost_function_revolute3(pid_params):
    global Kp_revolute3, Ki_revolute3, Kd_revolute3, cost_theta3, integral_theta3
    Kp_revolute3, Ki_revolute3, Kd_revolute3 = [max(p, 0) for p in pid_params]
    cost_theta3 = 0
    integral_theta3 = 0
    run_simulation()
    return cost_theta3

# Оптимизация PID параметров для каждого соединения
initial_pid_d = [1000, 1000, 700]
result_d = minimize(cost_function_prismatic, initial_pid_d, method='Nelder-Mead')
optimal_pid_d = result_d.x
Kp_prismatic, Ki_prismatic, Kd_prismatic = optimal_pid_d
print("Оптимальные PID для призматического соединения:", optimal_pid_d)

initial_pid_theta1 = [500, 500, 600]
result_theta1 = minimize(cost_function_revolute1, initial_pid_theta1, method='Nelder-Mead')
optimal_pid_theta1 = result_theta1.x
Kp_revolute1, Ki_revolute1, Kd_revolute1 = optimal_pid_theta1
print("Оптимальные PID для вращательного соединения 1:", optimal_pid_theta1)

initial_pid_theta2 = [120, 1000, 200]
result_theta2 = minimize(cost_function_revolute2, initial_pid_theta2, method='Nelder-Mead')
optimal_pid_theta2 = result_theta2.x
Kp_revolute2, Ki_revolute2, Kd_revolute2 = optimal_pid_theta2
print("Оптимальные PID для вращательного соединения 2:", optimal_pid_theta2)

initial_pid_theta3 = [100, 500, 600]
result_theta3 = minimize(cost_function_revolute3, initial_pid_theta3, method='Nelder-Mead')
optimal_pid_theta3 = result_theta3.x
Kp_revolute3, Ki_revolute3, Kd_revolute3 = optimal_pid_theta3
print("Оптимальные PID для вращательного соединения 3:", optimal_pid_theta3)

# Финальная симуляция с рендерингом
SC.renderer.Start()
if 'renderState' in exu.sys:
    SC.SetRenderState(exu.sys['renderState'])
SC.renderer.DoIdleTasks()

mbs.SolveDynamic(simulationSettings=simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)

SC.WaitForRenderEngineStopFlag()
exu.StopRenderer()
mbs.SolutionViewer()
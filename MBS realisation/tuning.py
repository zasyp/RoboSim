# trajectory_grey_pid_iterative.py

import os
import numpy as np
import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.robotics.motion import Trajectory, ProfilePTP
from helpful.constants import *
from utils.grey import GreyPIDController

# Параметры итерационного цикла
NUM_ITER = 5
LOG_FILE = 'pid_log.txt'

# Функция для запуска одной итерации симуляции
def run_simulation(pid_controllers):
    global m1
    global m2
    global m3
    # 1) Настройка системы
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    # Ground & bodies
    oGround = mbs.CreateGround(referencePosition=[0, -0.145, -0.805],
                               graphicsDataList=[box_graphics])
    iCilinder = RigidBodyInertia(mass=m_cil, inertiaTensor=inertiaTensorCilinder)
    i1 = RigidBodyInertia(mass=m1, inertiaTensor=inertiaTensor1)
    i2 = RigidBodyInertia(mass=m2, inertiaTensor=inertiaTensor2)
    i3 = RigidBodyInertia(mass=m3, inertiaTensor=inertiaTensor3)

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

    # Joints
    jointPrismatic = mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=joint0_pos,
                                              useGlobalFrame=True, axis=[0, 0, 1],  # Z-axis
                                              axisRadius=0.2 * w, axisLength=300 / 1000)
    jointRevolute1 = mbs.CreateRevoluteJoint(bodyNumbers=[b0, b1], position=joint1_pos,
                                             axis=[0, 0, 1], axisRadius=0.2 * w, axisLength=1 * w)
    jointRevolute2 = mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=joint2_pos,
                                             axis=[0, 0, 1], axisRadius=0.2 * w, axisLength=1 * w)
    jointRevolute3 = mbs.CreateRevoluteJoint(bodyNumbers=[b2, b3], position=joint3_pos,
                                             axis=[0, 0, 1], axisRadius=0.2 * w, axisLength=0.3 * w)

    # Sensors
    def add_sens(body, pos):
        sp = mbs.AddSensor(SensorBody(storeInternal=True, bodyNumber=body,
                                      localPosition=pos,
                                      outputVariableType=exu.OutputVariableType.Position))
        return sp

    sens0 = add_sens(b0, joint0_pos)
    sens1 = add_sens(b1, joint1_pos)
    sens2 = add_sens(b2, joint2_pos)
    sens3 = add_sens(b3, joint3_pos)

    # Trajectories
    q1 = [0.2, -2*np.pi/3, -2*np.pi/3]
    traj0 = Trajectory([0], 0)
    traj0.Add(ProfilePTP([q1[0]], maxVelocities=[1], maxAccelerations=[2], syncAccTimes=False))
    traj1 = Trajectory([0], 0)
    traj1.Add(ProfilePTP([q1[1]], maxVelocities=[0.5], maxAccelerations=[0.5], syncAccTimes=False))
    traj2 = Trajectory([q1[1]], 0)
    traj2.Add(ProfilePTP([q1[1]+q1[2]], maxVelocities=[0.5], maxAccelerations=[0.5], syncAccTimes=False))
    traj3 = Trajectory([q1[1]+q1[2]], 0)
    traj3.Add(ProfilePTP([q1[1]+q1[2]/2], maxVelocities=[0.5], maxAccelerations=[0.5], syncAccTimes=False))

    # Пользовательские функции нагрузки
    def make_load(marker, traj, pid, node, isPrism=False):
        def UF(mbs, t, loadVector):
            pos_des, vel_des, _ = traj.Evaluate(t)
            if isPrism:
                curr = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]
                vel  = mbs.GetNodeOutput(node, exu.OutputVariableType.Velocity)[2]
            else:
                curr = mbs.GetNodeOutput(node, exu.OutputVariableType.Rotation)[2]
                vel  = mbs.GetNodeOutput(node, exu.OutputVariableType.AngularVelocity)[2]
            err = pos_des[0] - curr
            derr= vel_des[0] - vel
            u = pid.update(err, derr, t)
            return [0,0,u]
        LoadCls = LoadForceVector if isPrism else LoadTorqueVector
        return mbs.AddLoad(LoadCls(markerNumber=marker, loadVector=[0,0,0],
                                   bodyFixed=not isPrism, loadVectorUserFunction=UF))

    # Маркеры
    m0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=joint0_pos))
    m1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=joint1_pos))
    m2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=joint2_pos))
    m3 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b3, localPosition=joint3_pos))

    # Добавляем нагрузки
    make_load(m0, traj0, pid_controllers[0], n0, isPrism=True)
    make_load(m1, traj1, pid_controllers[1], n1)
    make_load(m2, traj2, pid_controllers[2], n2)
    make_load(m3, traj3, pid_controllers[3], n3)

    # Симуляция
    settings = exu.SimulationSettings()
    settings.timeIntegration.numberOfSteps = int(3/1e-4)
    settings.timeIntegration.endTime = 3
    settings.timeIntegration.verboseMode = 0
    settings.timeIntegration.simulateInRealtime = False

    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings=settings)
    # Собираем данные ошибок
    def compute_rmse(sensor, traj, node, isPrism):
        data = mbs.GetSensorStoredData(sensor)
        times = data[:,0]
        vals = data[:,1]
        # Вычисляем желаемую траекторию на тех же times
        des = [traj.Evaluate(t)[0][0] for t in times]
        err = np.array(des) - vals
        return np.sqrt(np.mean(err**2))
    rmses = [
        compute_rmse(sens0, traj0, n0, True),
        compute_rmse(sens1, traj1, n1, False),
        compute_rmse(sens2, traj2, n2, False),
        compute_rmse(sens3, traj3, n3, False),
    ]
    return rmses

# Основной цикл
if __name__ == '__main__':
    # Инициализируем PID-контроллеры (4 звена)
    pids = [
        GreyPIDController(20,0.5,7),
        GreyPIDController(20,0.5,7),
        GreyPIDController(20,0.5,7),
        GreyPIDController(20,0.5,7),
    ]
    # Логируем стартовые коэффициенты
    with open(LOG_FILE, 'w') as f:
        f.write('iter,Kp_prism,Ki_prism,Kd_prism,Kp1,Ki1,Kd1,Kp2,Ki2,Kd2,Kp3,Ki3,Kd3\n')
    # Итерации
    for it in range(1, NUM_ITER+1):
        print(f'=== ИТЕРАЦИЯ {it} ===')
        rmses = run_simulation(pids)
        print('RMSE по звеньям:', rmses)
        # Сохраняем текущие коэффициенты
        with open(LOG_FILE, 'a') as f:
            coeffs = []
            for pid in pids:
                coeffs += [pid.Kp, pid.Ki, pid.Kd]
            f.write(','.join([str(it)] + [f'{c:.4f}' for c in coeffs]) + '\n')
    print('Готово! См. лог PID-коэффициентов в', LOG_FILE)

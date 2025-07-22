import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration, ProfilePTP

g = [0, -9.81, 0]
m = 10
a = 1
joint0_pos = [a/2,a/2,0]

bodyDim=[a, a, a]
p0 =    [0,0,0]
pMid0 = np.array([a*0.5,a*0.5,a*0.5])
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround = mbs.CreateGround(referencePosition=[0,0,0])
iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
iCube0 = iCube0.Translated([-0.25*a,0,0])
graphicsBody0 = graphics.Brick(centerPoint=[0,0,0],size=[a,a,a],
                               color=graphics.color.red)
graphicsCOM0 = graphics.Basis(origin=iCube0.com, length=a)
[n0, b0] = AddRigidBody(
    mainSys=mbs,
    inertia=iCube0,
    nodeType=str(exu.NodeType.RotationEulerParameters),
    position=p0,
    rotationMatrix=np.eye(3),
    gravity=g,
    graphicsDataList=[graphicsBody0])

jointPrismatic = mbs.CreatePrismaticJoint(bodyNumbers=[oGround, b0], position=joint0_pos,
                                          useGlobalFrame=True, axis=[1, 0, 0],  # Z-axis
                                          axisRadius=0.02*a, axisLength=20)


# Drive parameters (поменять согласно двигателю и редуктору)
Kp_d = 200    # N/m
Kd_d = 1e4      # N·s/m
maxF_d = 1e4    # N

q0 = [0, 0, 0]  # Initial [d1, theta1, theta2]
q1 = [90, 0, 0]  # Final [d1=0.1m, theta1=π/2, theta2=π/4]
trajectory = Trajectory(initialCoordinates=q0, initialTime=0)
trajectory.Add(ProfileConstantAcceleration(q1, duration=1))  # Move to q1 in 1s
trajectory.Add(ProfileConstantAcceleration(q1, duration=2))  # Hold for 2s
trajectory.Add(ProfilePTP(q1,syncAccTimes=False, maxVelocities=[1,1,1], maxAccelerations=[1,1,1]))


def drive_d1(mbs, t, load):
    # Считаем текущую координату и ошибку по позиции в линейном звене
    pos = mbs.GetNodeOutput(n0, exu.OutputVariableType.Coordinates)[2]  # d1 - 3-я координата
    err = abs(q1[0] - pos)
    tol = 1e-3  # допустимая погрешность в метрах

    if err > tol:
        # Пока не достигли нужной позиции — управление по траектории (feed-forward)
        u, _, _ = trajectory.Evaluate(t)
        return float(np.clip(u[0] * Kp_d, -maxF_d, maxF_d))
    else:
        # Если достигли цели — включаем ПД стабилизацию
        vel = mbs.GetNodeOutput(n0, exu.OutputVariableType.Coordinates_t)[2]
        err_dot = -vel
        control = Kp_d * (q1[0] - pos) + Kd_d * err_dot
        return float(np.clip(control, -maxF_d, maxF_d))

link0_marker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n0, coordinate=0))  # Z-position for prismatic joint

mbs.AddLoad(LoadCoordinate(markerNumber=link0_marker, load=0, loadUserFunction=drive_d1))


mbs.Assemble()
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 4 #simulation time
h = 1e-3 #step size
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
#simulationSettings.timeIntegration.simulateInRealtime = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms

SC.visualizationSettings.window.renderWindowSize = [1600,1200]
SC.visualizationSettings.openGL.multiSampling = 4  #improved OpenGL rendering
SC.visualizationSettings.general.autoFitScene = False

SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.showBasis = True #shows three RGB (=xyz) lines for node basis

SC.renderer.Start()
if 'renderState' in exu.sys: #reload old view
    SC.renderer.SetState(exu.sys['renderState'])

SC.renderer.DoIdleTasks()    #stop before simulating
mbs.SolveDynamic(simulationSettings = simulationSettings,
                 solverType = exu.DynamicSolverType.TrapezoidalIndex2)
mbs.SolveDynamic(simulationSettings = simulationSettings)
SC.renderer.DoIdleTasks()   #stop before closing
SC.renderer.Stop()          #safely close rendering window!
mbs.SolutionViewer()
#alternatively, we could load solution from a file:
#from exudyn.utilities import LoadSolutionFile
#sol = LoadSolutionFile('coordinatesSolution.txt')
#mbs.SolutionViewer(sol)

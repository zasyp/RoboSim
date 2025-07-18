import exudyn as exu
#import specific items, inertia class for general cuboid (hexahedral) block, etc.
from exudyn.utilities import *
import exudyn.graphics as graphics #only import as graphics if it does not conflict
import numpy as np #for postprocessing

SC = exu.SystemContainer()
mbs = SC.AddSystem()

g = [0, -9.81, 0]

l1 = 0.205  # Звено 1
l2 = 0.205  # Звено 2
l3 = 0.39   # Звено 3

m1 = 6.936  # кг, Звено 1
m2 = 7.609  # кг, Звено 2
m3 = 0.533  # кг, Звено 3

w = 0.1

com1_global = np.array([0,0,0])  # Звено 1
com2_global = np.array([-205/1000, 0, 81 / 1000])  # Звено 2
com3_global = np.array([-410 / 1000, 0 / 1000, 155 / 1000])  # Звено 3

joint0_pos = np.array([0, 0, 0])          # Шарнир 0 (основание)
joint1_pos = np.array([-l1, 0, com2_global[2]])         # Шарнир 1 (между звеном 1 и 2)
joint2_pos = np.array([-(l1 + l2), 0, com3_global[2]])    # Шарнир 2 (между звеном 2 и 3)

com1_local = com1_global - joint0_pos
com2_local = com2_global - joint1_pos
com3_local = com3_global - joint2_pos

inertiaTensor1 = np.array([
    [25749603.700 * 1e3, 25062.125 * 1e3, 9651359.305 * 1e3],
    [25062.125 * 1e3, 92340828.791 * 1e3, 24778.147 * 1e3],
    [9651359.305 * 1e3, 24778.147 * 1e3, 83172050.999 * 1e3]
])

inertiaTensor2 = np.array([
    [13449632.937 * 1e3, -24026.329 * 1e3, 1660746.774 * 1e3],
    [-24026.329 * 1e3, 84912103.395 * 1e3, 27847.617 * 1e3],
    [1660746.774 * 1e3, 27847.617 * 1e3, 89804961.922 * 1e3]
])

inertiaTensor3 = np.array([
    [480197.752 * 1e3, 835.157 * 1e3, 2908.542 * 1e3],
    [835.157 * 1e3, 14103107.261 * 1e3, 0 * 1e3],
    [2908.542 * 1e3, 0 * 1e3, 14577958.019 * 1e3]
])

oGround = mbs.CreateGround(referencePosition=[0,0,0])

graphicsBody1 = graphics.FromSTLfile('solution/link1.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)
graphicsBody2 = graphics.FromSTLfile('solution/link2.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)
graphicsBody3 = graphics.FromSTLfile('solution/link3.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)

i1 = RigidBodyInertia(mass=m1, inertiaTensor=inertiaTensor1)
i2 = RigidBodyInertia(mass=m2, inertiaTensor=inertiaTensor2)
i3 = RigidBodyInertia(mass=m3, inertiaTensor=inertiaTensor3)

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

markerGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=joint0_pos))

mbs.CreateRevoluteJoint(bodyNumbers=[oGround, b1], position=joint0_pos,
                        axis=[0,0,1], axisRadius=0.2*w, axisLength=1*w)

mbs.CreateRevoluteJoint(bodyNumbers=[b1, b2], position=joint1_pos,
                        axis=[0,0,1], axisRadius=0.2*w, axisLength=1*w)

mbs.CreateRevoluteJoint(bodyNumbers=[b2, b3], position=joint2_pos,
                        axis=[0,0,1], axisRadius=0.2*w, axisLength=0.3*w)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

torque = [0,0,2e9]
force = [0,2e9,0]
mbs.CreateForce(bodyNumber=b1,
                loadVector=force,
                localPosition=[-l1-0.1, 0, com2_global[2]+0.05], #at tip
                bodyFixed=False) #if True, direction would corotate with body
mbs.CreateTorque(bodyNumber=b1,
                loadVector=torque,
                localPosition=[0,0,0],   #at body's reference point/center
                bodyFixed=False)

mbs.Assemble()

tEnd = 10 #simulation time
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

exu.StartRenderer()
if 'renderState' in exu.sys: #reload old view
    SC.SetRenderState(exu.sys['renderState'])

mbs.WaitForUserToContinue() #stop before simulating
mbs.SolveDynamic(simulationSettings = simulationSettings,
                 solverType = exu.DynamicSolverType.TrapezoidalIndex2)
mbs.SolveDynamic(simulationSettings = simulationSettings)
SC.WaitForRenderEngineStopFlag() #stop before closing
exu.StopRenderer() #safely close rendering window!
mbs.SolutionViewer()
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.robotics import *
import numpy as np
from exudyn.kinematicTree import KinematicTree66, JointTransformMotionSubspace
from helpful.constants import *

graphicsBaseList = [graphicsBodyBox, graphicsBodyCylinder, graphicsBody1, graphicsBody2, graphicsBody3]

toolGraphics = [graphics.Basis(length=0.3*0)]

robot = Robot(
    gravity=g,
    base=RobotBase()
)

visualisationBox = VRobotLink(
    graphicsData=['graphicsBodyBox']
)

visualisationCylinder = VRobotLink(
    graphicsData=['graphicsBodyCylinder']
)

visualisationLink1 = VRobotLink(
    graphicsData=['graphicsBody1']
)

visualisationLink2 = VRobotLink(
    graphicsData=['graphicsBody2']
)

visualisationLink3 = VRobotLink(
    graphicsData=['graphicsBody3']
)

linkBox = RobotLink(
    mass=m_box,
    COM=[0,0,0],
    inertia=inertiaTensorBox,
    parent=-1,
    visualization=visualisationBox
)

linkCylinder = RobotLink(
    mass=m_cyl,
    COM=com_cyl_global,
    inertia=inertiaTensorCilinder,
    jointType='Pz',
    parent=linkBox,
    visualization=visualisationCylinder,
)

link1 = RobotLink(
    mass=m1,
    COM=com1_global,
    inertia=inertiaTensor1,
    jointType='Rz',
    parent=linkCylinder,
    visualization=visualisationLink1,
)

link2 = RobotLink(
    mass=m2,
    COM=com2_global,
    inertia=inertiaTensor2,
    jointType='Rz',
    parent=link1,
    visualization=visualisationLink2,
)

link3 = RobotLink(
    mass=m3,
    COM=com2_global,
    inertia=inertiaTensor3,
    jointType='Rz',
    parent=link1,
    visualization=visualisationLink3,
)
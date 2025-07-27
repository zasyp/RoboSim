import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
import exudyn.graphics as graphics
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.robotics import *
import numpy as np
from exudyn.kinematicTree import KinematicTree66, JointTransformMotionSubspace
from exudyn.robotics.models import LinkDict2Robot
from helpful.constants import *

link0_visualization = VRobotLink(
    graphicsData=['graphicsBodyCilinder'],
    linkColor = [0.4,0.4,0.4,1],
    jointRadius=0.03,
    jointWidth=0.3
)

link0 = RobotLink(
    mass=m_cil,
    inertia=inertiaTensorCilinder,
    jointType='Rz',
    parent=-2,
    visualization=link0_visualization,
    PDcontrol=(1,1)
)

from exudyn import *
import exudyn.graphics as graphics
from exudyn.graphicsDataUtilities import *
import numpy as np

g = [0, -9.81, 0]
l1 = 0.205
l2 = 0.205
l3 = 0.39
m1 = 6.936
m2 = 7.609
m3 = 0.533
m_cyl = 9.838
m_box = 177.584
w = 0.1

com_cyl_global = np.array([0,0,-300/1000])
com1_global = np.array([0,0,0])
com2_global = np.array([-205/1000, 0, 381 / 1000])
com3_global = np.array([-410 / 1000, 0 / 1000, 455 / 1000])
joint0_pos = np.array([0, 0, 0])
joint1_pos = np.array([0, 0, 300])
joint2_pos = np.array([-l1, 0, com2_global[2]])
joint3_pos = np.array([-(l1 + l2), 0, com3_global[2]])

graphicsBodyBox = graphics.FromSTLfile('../graphics/box.stl',
                                    color=graphics.color.blue,
                                    scale=0.001)
graphicsBody1 = graphics.FromSTLfile('../graphics/link1.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)
graphicsBody2 = graphics.FromSTLfile('../graphics/link2.stl',
                                     color=graphics.color.red,
                                     scale=0.001)
graphicsBody3 = graphics.FromSTLfile('../graphics/link3.stl',
                                     color=graphics.color.dodgerblue,
                                     scale=0.001)
graphicsBodyCylinder = graphics.FromSTLfile('../graphics/cylinder.stl',
                                            color=graphics.color.dodgerblue,
                                            scale=0.001)

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
inertiaTensorBox = np.array([
    [12827734685.081602 / 1e9, -141913220.479312 / 1e9, 56073179.720621 / 1e9],
    [-141913220.479312 / 1e9, 15642337808.970053 / 1e9, 66762286.742506 / 1e9],
    [56073179.720621 / 1e9, 66762286.742506 / 1e9, 9218564386.912329 / 1e9]
])

preHT_Cyl = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0.145],
    [0, 0, 1, 0.505],
    [0, 0, 0, 1]
])
preHT_1 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0.3],
    [0, 0, 0, 1]
])
preHT_2 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0.205],
    [0, 0, 1, 0.081],
    [0, 0, 0, 1]
])
preHT_3 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0.205],
    [0, 0, 1, 0.074],
    [0, 0, 0, 1]
])
HT_tool = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0.39],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
# Set PD control gains for joints that require control
kp_trans = 1e100  # Prismatic joint proportional gain [N/m]
kd_trans = 1e100  # Prismatic joint derivative gain [N·s/m]
kp_rot = 1e100    # Revolute joint proportional gain [Nm/rad]
kd_rot = 1e100     # Revolute joint derivative gain [Nm·s/rad]
kp_rot2 = 1e100
kd_rot2 = 1e100
spring = 0
spring_l1l2 = spring
damper = 10
nLinks = 4
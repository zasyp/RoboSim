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
m_cil = 9.838
m_box = 177.584

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

graphicsBodyCilinder = graphics.FromSTLfile('../graphics/cilinder.stl',
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


spring = 30
spring_l1l2 = 20 #stiffness of spring-damper in N/m
damper = 8
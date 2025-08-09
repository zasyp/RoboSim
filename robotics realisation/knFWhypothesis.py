import os
import numpy as np
import matplotlib.pyplot as plt
import exudyn as exu
from exudyn.utilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.robotics import *
from exudyn.robotics.motion import Trajectory, ProfileConstantAcceleration
from exudyn.robotics.special import *
from helpful.constants import *

# Visualization definitions
vis_box = VRobotBase(graphicsData=[graphicsBodyBox])
vis_cyl = VRobotLink(graphicsData=[graphicsBodyCylinder])
vis_link1 = VRobotLink(graphicsData=[graphicsBody1])
vis_link2 = VRobotLink(graphicsData=[graphicsBody2])
vis_link3 = VRobotLink(graphicsData=[graphicsBody3])

# Constants
useKT = True
q0 = np.array([0, 0, 0, 0])
factorValue1 = -0.5
g = np.array([0, 0, -9.81])  # Assuming gravity from constants
output_dir = "sensor_outputs"
os.makedirs(output_dir, exist_ok=True)
tEnd = 10
h = 0.25e-3


def create_robot():
    base = RobotBase(visualization=vis_box)
    robot = Robot(gravity=g, base=base, tool=RobotTool(HT=HT_tool))

    robot.AddLink(RobotLink(mass=m_cyl, COM=com_cyl_global, inertia=inertiaTensorCylinder,
                            jointType='Pz', parent=-2, preHT=preHT_Cyl,
                            visualization=vis_cyl, PDcontrol=(kp_trans, kd_trans)))

    robot.AddLink(RobotLink(mass=m1, COM=joint1_pos, inertia=inertiaTensor1,
                            jointType='Rz', parent=-2, preHT=preHT_1,
                            visualization=vis_link1, PDcontrol=(kp_rot, kd_rot)))

    robot.AddLink(RobotLink(mass=m2, COM=joint2_pos, inertia=inertiaTensor2,
                            jointType='Rz', parent=-2, preHT=preHT_2,
                            visualization=vis_link2, PDcontrol=(kp_rot2, kd_rot2)))

    robot.AddLink(RobotLink(mass=m3, COM=joint3_pos, inertia=inertiaTensor3,
                            jointType='Rz', parent=-2, preHT=preHT_3,
                            visualization=vis_link3, PDcontrol=(0, 0)))

    return robot


def create_trajectory():
    traj = Trajectory(initialCoordinates=q0, initialTime=0)
    traj.Add(ProfileConstantAcceleration([0.1, -0.5 * np.pi, 0.3 * np.pi, 0], 2))
    traj.Add(ProfileConstantAcceleration([0.2, 0.5 * np.pi, -0.3 * np.pi, 0], 2))
    traj.Add(ProfileConstantAcceleration([0.1, -0.5 * np.pi, -0.1 * np.pi, 0], 2))
    traj.Add(ProfileConstantAcceleration([0.3, -0.3 * np.pi, -0.4 * np.pi, 0], 2))
    traj.Add(ProfileConstantAcceleration([0, 0, 0, 0], 2))
    return traj

def joint_motion_subspace(joint_type):
    if joint_type == 'Px': return np.array([0,0,0,1,0,0])
    if joint_type == 'Py': return np.array([0,0,0,0,1,0])
    if joint_type == 'Pz': return np.array([0,0,0,0,0,1])
    if joint_type == 'Rx': return np.array([1,0,0,0,0,0])
    if joint_type == 'Ry': return np.array([0,1,0,0,0,0])
    if joint_type == 'Rz': return np.array([0,0,1,0,0,0])
    raise ValueError(f"Unknown joint type: {joint_type}")

def RNEA(q, q_t, q_tt, mbs, oKT, robot, g):
    # Number of links (including base if present)
    N_B = robot.NumberOfLinks()
    v = np.zeros((N_B, 6))      # spatial velocity: [omega(3); v(3)]
    a = np.zeros((N_B, 6))      # spatial acceleration: [alpha(3); a(3)]
    a[:, 3:] = -g               # set linear acceleration part to -g (gravity)
    f = np.zeros((N_B, 6))      # spatial force: [moment(3); force(3)]
    tau = np.zeros(N_B)         # joint torques/forces

    # Get parameters from ObjectKinematicTree
    link_parents = mbs.GetObjectParameter(oKT, 'linkParents')
    link_masses = mbs.GetObjectParameter(oKT, 'linkMasses')
    link_inertias = mbs.GetObjectParameter(oKT, 'linkInertiasCOM')

    # Forward pass: compute spatial velocities and accelerations
    for i in range(N_B):
        # get parent spatial state (if any)
        parent = link_parents[i]
        if parent != -1:
            v[i] = v[parent].copy()
        else:
            v[i] = np.zeros(6)

        # add joint contribution to velocity
        if i > 0:
            phi = joint_motion_subspace(robot.links[i].jointType)  # 6-vector
            v[i] += phi * q_t[i]

        # accelerations: start from parent
        if parent != -1:
            a[i] = a[parent].copy()
        else:
            a[i] = np.zeros(6)
            a[i][3:] = -g  # ensure gravity on base

        # add joint contributions to acceleration
        if i > 0:
            phi = joint_motion_subspace(robot.links[i].jointType)
            # angular part: alpha contribution; plus coriolis-like term phi x (omega * qdot)
            # here phi[:3] are angular components, phi[3:] linear components
            a[i][:3] += phi[:3] * q_tt[i] + np.cross(v[i][:3], phi[:3] * q_t[i])
            a[i][3:] += phi[3:] * q_tt[i] + np.cross(v[i][:3], phi[3:] * q_t[i])

        # spatial inertia * spatial acceleration -> spatial force
        # approximate rotational moment = I * alpha (link_inertias are 3x3 for COM)
        moment = link_inertias[i].dot(a[i][:3])
        force  = link_masses[i] * a[i][3:]
        f[i] = np.hstack((moment, force))

    # Backward pass: propagate forces and compute joint torques
    for i in range(N_B-1, -1, -1):
        if i > 0:
            # joint motion subspace for this joint
            phi = joint_motion_subspace(robot.links[i].jointType)  # 6-vector
            # joint torque/force is projection of spatial force onto phi
            tau[i] = phi.dot(f[i])
        else:
            tau[i] = 0.0

        parent = link_parents[i]
        if parent != -1:
            # Transform f[i] to parent frame before accumulation.
            # NOTE: the exact transform depends on how you obtain HTs (robot.JointHT etc.)
            # Keep the existing method for computing parent_to_i if it works in your setup,
            # but ensure it yields a 6x6 spatial transform matrix X_parent_child.
            try:
                parent_to_i = robot.JointHT(q)[i].T.dot(robot.JointHT(q)[parent])
                # If parent_to_i is a 4x4 homogeneous transform, you must convert it
                # into a 6x6 spatial transform X before applying to a 6-vector f.
                # Here we assume parent_to_i is already the correct 6x6 spatial transform.
                f[parent] += parent_to_i.dot(f[i])
            except Exception:
                # Fallback: if transform is not ready/available, just accumulate forces in parent frame
                # This is a safe fallback but may be incorrect if frames differ.
                f[parent] += f[i]

    return tau

def pre_step_uf(mbs, t, robot, traj, oKT, torque_values):
    if useKT:
        q = mbs.GetObjectOutputBody(oKT, exu.OutputVariableType.Coordinates, [0, 0, 0])
        u, v, a = traj.Evaluate(t)
        q_t = v
        q_tt = a
        static_torques = RNEA(q, q_t, q_tt, mbs, oKT, robot, g)
        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
        mbs.SetObjectParameter(oKT, 'jointForceVector', static_torques)
        torque_values.append(static_torques)
    return True

def add_sensors(mbs, oKT, output_dir):
    sensors = {}
    vars = [
        ('verticalDisp', 0, exu.OutputVariableType.Displacement, 'verticalDisp.txt', 3),
        ('verticalVel', 0, exu.OutputVariableType.VelocityLocal, 'verticalVel.txt', 3),
        ('verticalAcc', 0, exu.OutputVariableType.AccelerationLocal, 'verticalAcc.txt', 3),
        ('theta1', 1, exu.OutputVariableType.Rotation, 'theta1_deg.txt', 3),
        ('omega1', 1, exu.OutputVariableType.AngularVelocityLocal, 'omega1_deg.txt', 3),
        ('epsilon1', 1, exu.OutputVariableType.AngularAccelerationLocal, 'epsilon1_deg.txt', 3),
        ('theta2', 2, exu.OutputVariableType.Rotation, 'theta2_deg.txt', 3),
        ('omega2', 2, exu.OutputVariableType.AngularVelocityLocal, 'omega2_deg.txt', 3),
        ('epsilon2', 2, exu.OutputVariableType.AngularAccelerationLocal, 'epsilon2_deg.txt', 3),
        ('theta3', 3, exu.OutputVariableType.Rotation, 'theta3_deg.txt', 3),
        ('omega3', 3, exu.OutputVariableType.AngularVelocityLocal, 'omega3_deg.txt', 3),
        ('epsilon3', 3, exu.OutputVariableType.AngularAccelerationLocal, 'epsilon3_deg.txt', 3)
    ]
    for name, link, var_type, filename, comp in vars:
        sensors[name] = mbs.AddSensor(SensorKinematicTree(
            objectNumber=oKT, linkNumber=link, localPosition=[0, 0, 0],
            outputVariableType=var_type, storeInternal=True, writeToFile=True,
            fileName=os.path.join(output_dir, filename), name=name))
    return sensors


def setup_simulation(robot, traj):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    robot_dict = robot.CreateKinematicTree(mbs=mbs, name="WHR")
    oKT = robot_dict['objectKinematicTree']

    mJoint2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=2))
    mJoint3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=3))
    mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mJoint3, mJoint2], factorValue1=factorValue1))

    torque_values = []
    mbs.SetPreStepUserFunction(lambda mbs, t: pre_step_uf(mbs, t, robot, traj, oKT, torque_values))

    sensors = add_sensors(mbs, oKT, output_dir)
    mbs.Assemble()

    sim_settings = exu.SimulationSettings()
    sim_settings.timeIntegration.numberOfSteps = int(tEnd / h)
    sim_settings.timeIntegration.endTime = tEnd
    sim_settings.timeIntegration.verboseMode = 1
    sim_settings.solutionSettings.solutionWritePeriod = 0.005
    sim_settings.solutionSettings.sensorsWritePeriod = 0.005

    # Visualization settings
    SC.visualizationSettings.window.renderWindowSize = (1600, 1200)
    SC.visualizationSettings.openGL.multiSampling = 4
    SC.visualizationSettings.general.autoFitScene = False
    SC.visualizationSettings.nodes.drawNodesAsPoint = False
    SC.visualizationSettings.nodes.showBasis = True
    SC.visualizationSettings.general.drawWorldBasis = True
    SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
    SC.visualizationSettings.openGL.lineWidth = 2
    SC.visualizationSettings.openGL.light0position = (-6, 2, 12, 0)

    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys['renderState'])

    return SC, mbs, sim_settings, torque_values, sensors


def run_simulation(SC, mbs, sim_settings):
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()  # Replace WaitForUserToContinue
    mbs.SolveDynamic(simulationSettings=sim_settings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
    exu.StopRenderer()
    mbs.SolutionViewer()


def get_sensor_data(mbs, sensors):
    data = {name: mbs.GetSensorStoredData(sid) for name, sid in sensors.items()}
    return data


def process_data(data, traj):
    times = data['theta1'][:, 0]
    n = len(times)

    actual = {
        'pos': np.column_stack((data['verticalDisp'][:, 3], data['theta1'][:, 3],
                                data['theta2'][:, 3] - data['theta1'][:, 3],
                                data['theta3'][:, 3] - data['theta2'][:, 3] - data['theta1'][:, 3])),
        'vel': np.column_stack((data['verticalVel'][:, 3], data['omega1'][:, 3],
                                data['omega2'][:, 3] - data['omega1'][:, 3],
                                data['omega3'][:, 3] - data['omega2'][:, 3] - data['omega1'][:, 3])),
        'acc_sensor': np.column_stack((data['verticalAcc'][:, 3], data['epsilon1'][:, 3],
                                       data['epsilon2'][:, 3] - data['epsilon1'][:, 3],
                                       data['epsilon3'][:, 3] - data['epsilon2'][:, 3] - data['epsilon1'][:, 3]))
    }

    actual['acc_calc'] = np.column_stack((
        np.gradient(actual['vel'][:, 0], times),
        np.gradient(actual['vel'][:, 1], times),
        np.gradient(actual['vel'][:, 2], times),
        np.gradient(actual['vel'][:, 3], times)
    ))

    ideal = {'pos': np.zeros((n, 4)), 'vel': np.zeros((n, 4)), 'acc': np.zeros((n, 4))}
    for i, t in enumerate(times):
        u, v, a = traj.Evaluate(t)
        ideal['pos'][i] = u
        ideal['vel'][i] = v
        ideal['acc'][i] = a

    z0 = data['verticalDisp'][0, 3]
    ideal['pos'][:, 0] += z0

    return times, actual, ideal


def plot_all(times, actual, ideal):
    labels = ['Vertical', 'Theta1/Omega1/Epsilon1', 'Theta2/Omega2/Epsilon2', 'Theta3/Omega3/Epsilon3']
    units = [('m', 'm/s', 'm/s²'), ('rad', 'rad/s', 'rad/s²')] * 2

    fig, axs = plt.subplots(3, 4, figsize=(20, 15))
    for row, key in enumerate(['pos', 'vel', 'acc']):
        for col in range(4):
            ax = axs[row, col]
            ax.plot(times, actual[key if key != 'acc' else 'acc_calc'][:, col], 'b-', label='Actual/Calculated')
            if key != 'acc':
                ax.plot(times, ideal[key][:, col if col < 3 else 3], 'r--', label='Ideal')
            else:
                ax.plot(times, ideal['acc'][:, col if col < 3 else 3], 'r--', label='Ideal')
            ax.set_title(f'{labels[col]} {["Position", "Velocity", "Acceleration"][row]} ({units[col][row]})')
            ax.legend()
            ax.grid()

    plt.tight_layout(pad=2.0)
    plt.savefig('all_sensors_data_with_ideal.png')
    plt.close()


def plot_errors(times, actual, ideal):
    fig, axs = plt.subplots(3, 3, figsize=(15, 15))
    for row, key in enumerate(['pos', 'vel', 'acc']):
        for col in range(3):
            ax = axs[row, col]
            err = ideal[key][:, col] - actual[key if key != 'acc' else 'acc_calc'][:, col]
            ax.plot(times, err)
            ax.set_title(
                f'{["Vertical", "Theta1", "Theta2"][col]} {["Position", "Velocity", "Acceleration"][row]} Error')
            ax.set_ylabel('Error')
            if row == 2:
                ax.set_xlabel('Time (s)')
            ax.grid()

    plt.tight_layout()
    plt.savefig('all_errors.png')
    plt.close()


def plot_torques(torque_values):
    torques = np.array(torque_values)
    times = np.arange(len(torques)) * (tEnd / len(torques))
    plt.figure(figsize=(12, 8))
    for i, label in enumerate(['Cylinder', 'Link1', 'Link2', 'Link3']):
        plt.plot(times, torques[:, i], label=label)
    plt.title('Force and Torques in Links')
    plt.xlabel('Time (s)')
    plt.ylabel('Torque (N·m)')
    plt.legend()
    plt.grid()
    plt.savefig('torques.png')
    plt.close()


def save_debug_data(torque_values, output_dir):
    with open(os.path.join(output_dir, "Torques.txt"), "w") as f:
        for tau in torque_values:
            f.write(f"{tau}\n")


# Main execution
robot = create_robot()
traj = create_trajectory()
SC, mbs, sim_settings, torque_values, sensors = setup_simulation(robot, traj)
run_simulation(SC, mbs, sim_settings)
data = get_sensor_data(mbs, sensors)
times, actual, ideal = process_data(data, traj)
plot_all(times, actual, ideal)
plot_errors(times, actual, ideal)
plot_torques(torque_values)
save_debug_data(torque_values, output_dir)
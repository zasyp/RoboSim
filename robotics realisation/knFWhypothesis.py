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
g = np.array([0, 0, -9.81])  # Ensure g is a NumPy array
output_dir = "sensor_outputs"
os.makedirs(output_dir, exist_ok=True)
tEnd = 6
h = 0.25e-3


def create_robot():
    """Create robot with proper link hierarchy and visualization."""
    base = RobotBase(visualization=vis_box)
    robot = Robot(gravity=g, base=base, tool=RobotTool(HT=HT_tool))

    robot.AddLink(RobotLink(
        mass=m_cyl, COM=com_cyl_global, inertia=inertiaTensorCylinder,
        jointType='Pz', parent=-1, preHT=preHT_Cyl,
        visualization=vis_cyl, PDcontrol=(kp_trans, kd_trans)
    ))

    robot.AddLink(RobotLink(
        mass=m1, COM=joint1_pos, inertia=inertiaTensor1,
        jointType='Rz', parent=0, preHT=preHT_1,
        visualization=vis_link1, PDcontrol=(kp_rot, kd_rot)
    ))

    robot.AddLink(RobotLink(
        mass=m2, COM=joint2_pos, inertia=inertiaTensor2,
        jointType='Rz', parent=1, preHT=preHT_2,
        visualization=vis_link2, PDcontrol=(kp_rot2, kd_rot2)
    ))

    robot.AddLink(RobotLink(
        mass=m3, COM=joint3_pos, inertia=inertiaTensor3,
        jointType='Rz', parent=2, preHT=preHT_3,
        visualization=vis_link3, PDcontrol=(0, 0)
    ))

    return robot


def create_trajectory():
    """Create trajectory with constant acceleration profiles."""
    traj = Trajectory(initialCoordinates=q0, initialTime=0)
    traj.Add(ProfileConstantAcceleration([0.1, -0.5 * np.pi, 0.3 * np.pi, 0], 1))
    traj.Add(ProfileConstantAcceleration([0.2, 0.5 * np.pi, -0.3 * np.pi, 0], 1))
    traj.Add(ProfileConstantAcceleration([0.1, -0.5 * np.pi, -0.1 * np.pi, 0], 1))
    traj.Add(ProfileConstantAcceleration([0.3, -0.3 * np.pi, -0.4 * np.pi, 0], 1))
    traj.Add(ProfileConstantAcceleration([0, 0, 0, 0], 1))
    return traj


def joint_motion_subspace(joint_type):
    """Convert joint type to 6D spatial motion subspace vector."""
    if joint_type == 'Px': return np.array([0, 0, 0, 1, 0, 0])
    if joint_type == 'Py': return np.array([0, 0, 0, 0, 1, 0])
    if joint_type == 'Pz': return np.array([0, 0, 0, 0, 0, 1])
    if joint_type == 'Rx': return np.array([1, 0, 0, 0, 0, 0])
    if joint_type == 'Ry': return np.array([0, 1, 0, 0, 0, 0])
    if joint_type == 'Rz': return np.array([0, 0, 1, 0, 0, 0])
    raise ValueError(f"Unknown joint type: {joint_type}")


def skew(v):
    """Create skew-symmetric matrix from 3D vector."""
    return np.array([[0, -v[2], v[1]],
                     [v[2], 0, -v[0]],
                     [-v[1], v[0], 0]])


def RNEA(q, q_t, q_tt, mbs, oKT, robot, g):
    """
    Recursive Newton-Euler Algorithm for computing joint torques/forces.

    Parameters:
    -----------
    q : array - joint positions
    q_t : array - joint velocities
    q_tt : array - joint accelerations
    mbs : multibody system
    oKT : kinematic tree object number
    robot : Robot object
    g : gravity vector

    Returns:
    --------
    tau : array of joint torques/forces
    """
    N_B = robot.NumberOfLinks()
    v = [np.zeros(6) for _ in range(N_B)]
    a = [np.zeros(6) for _ in range(N_B)]
    f = [np.zeros(6) for _ in range(N_B)]

    # Get robot parameters
    link_parents = mbs.GetObjectParameter(oKT, 'linkParents')
    link_masses = mbs.GetObjectParameter(oKT, 'linkMasses')
    link_inertias = mbs.GetObjectParameter(oKT, 'linkInertiasCOM')

    # Precompute transformations
    HT_all = robot.JointHT(q)

    # Forward pass: velocities and accelerations
    for i in range(N_B):
        parent = link_parents[i]

        if parent != -1:
            v[i] = v[parent].copy()
            a[i] = a[parent].copy()
        else:
            a[i][3:] = -g  # gravity on base

        # Add joint contribution
        joint_type = robot.links[i].jointType
        phi = joint_motion_subspace(joint_type)

        v[i] += phi * q_t[i]
        a[i][:3] += phi[:3] * q_tt[i] + np.cross(v[i][:3], phi[:3] * q_t[i])
        a[i][3:] += phi[3:] * q_tt[i] + np.cross(v[i][:3], phi[3:] * q_t[i])

        # Compute spatial force
        omega = v[i][:3]
        alpha = a[i][:3]
        moment = link_inertias[i].dot(alpha) + np.cross(omega, link_inertias[i].dot(omega))
        force = link_masses[i] * a[i][3:]
        f[i] = np.hstack((moment, force))

    # Backward pass: torques
    tau = np.zeros(N_B)
    for i in range(N_B - 1, -1, -1):
        joint_type = robot.links[i].jointType
        phi = joint_motion_subspace(joint_type)
        tau[i] = phi.dot(f[i])

        # Propagate force to parent
        if i > 0:
            parent = link_parents[i]
            if parent != -1:
                HT_parent_to_i = np.linalg.inv(HT_all[parent]) @ HT_all[i]
                R = HT_parent_to_i[:3, :3]
                p = HT_parent_to_i[:3, 3]

                X = np.block([
                    [R, skew(p) @ R],
                    [np.zeros((3, 3)), R]
                ])
                f[parent] += X.T @ f[i]

    return tau


def pre_step_uf(mbs, t, robot, traj, oKT, torque_values):
    """Pre-step user function for dynamic control."""
    if useKT:
        q = mbs.GetObjectOutputBody(oKT, exu.OutputVariableType.Coordinates, [0, 0, 0])
        u, v, a = traj.Evaluate(t)
        torques = RNEA(q, v, a, mbs, oKT, robot, g)

        mbs.SetObjectParameter(oKT, 'jointPositionOffsetVector', u)
        mbs.SetObjectParameter(oKT, 'jointVelocityOffsetVector', v)
        mbs.SetObjectParameter(oKT, 'jointForceVector', torques)

        torque_values.append(torques.copy())  # Store a copy

    return True


def add_sensors(mbs, oKT, output_dir):
    """Add sensors for all links."""
    sensors = {}
    sensor_defs = [
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

    for name, link, var_type, filename, comp in sensor_defs:
        sensors[name] = mbs.AddSensor(SensorKinematicTree(
            objectNumber=oKT, linkNumber=link, localPosition=[0, 0, 0],
            outputVariableType=var_type, storeInternal=True, writeToFile=True,
            fileName=os.path.join(output_dir, filename), name=name))

    return sensors


def setup_simulation(robot, traj):
    """Setup the simulation environment."""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    robot_dict = robot.CreateKinematicTree(mbs=mbs, name="WHR")
    oKT = robot_dict['objectKinematicTree']

    # Add constraint
    mJoint2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=2))
    mJoint3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=0, coordinate=3))
    mbs.AddObject(ObjectConnectorCoordinate(
        markerNumbers=[mJoint3, mJoint2], factorValue1=factorValue1))

    # Store torque values
    torque_values = []

    # Set pre-step function with closure
    def pre_step_wrapper(mbs, t):
        return pre_step_uf(mbs, t, robot, traj, oKT, torque_values)

    mbs.SetPreStepUserFunction(pre_step_wrapper)

    # Add sensors
    sensors = add_sensors(mbs, oKT, output_dir)
    mbs.Assemble()

    # Simulation settings
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

    # Restore render state if available
    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys['renderState'])

    return SC, mbs, sim_settings, torque_values, sensors


def run_simulation(SC, mbs, sim_settings):
    """Run the dynamic simulation."""
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()  # Replace deprecated WaitForUserToContinue
    mbs.SolveDynamic(simulationSettings=sim_settings,
                     solverType=exu.DynamicSolverType.TrapezoidalIndex2)
    exu.StopRenderer()
    mbs.SolutionViewer()


def get_sensor_data(mbs, sensors):
    """Retrieve stored sensor data."""
    data = {name: mbs.GetSensorStoredData(sid) for name, sid in sensors.items()}
    return data


def process_data(data, traj):
    """Process sensor data and generate ideal trajectory."""
    times = data['theta1'][:, 0]
    n = len(times)

    # Extract actual data with proper relative angles
    actual = {
        'pos': np.column_stack([
            data['verticalDisp'][:, 3],  # z-position
            data['theta1'][:, 3],  # theta1
            data['theta2'][:, 3] - data['theta1'][:, 3],  # theta2 relative
            data['theta3'][:, 3] - data['theta2'][:, 3] - data['theta1'][:, 3]  # theta3 relative
        ]),
        'vel': np.column_stack([
            data['verticalVel'][:, 3],
            data['omega1'][:, 3],
            data['omega2'][:, 3] - data['omega1'][:, 3],
            data['omega3'][:, 3] - data['omega2'][:, 3] - data['omega1'][:, 3]
        ]),
        'acc_sensor': np.column_stack([
            data['verticalAcc'][:, 3],
            data['epsilon1'][:, 3],
            data['epsilon2'][:, 3] - data['epsilon1'][:, 3],
            data['epsilon3'][:, 3] - data['epsilon2'][:, 3] - data['epsilon1'][:, 3]
        ])
    }

    # Calculate accelerations via numerical differentiation
    actual['acc_calc'] = np.column_stack([
        np.gradient(actual['vel'][:, 0], times),
        np.gradient(actual['vel'][:, 1], times),
        np.gradient(actual['vel'][:, 2], times),
        np.gradient(actual['vel'][:, 3], times)
    ])

    # Generate ideal trajectory
    ideal = {'pos': np.zeros((n, 4)), 'vel': np.zeros((n, 4)), 'acc': np.zeros((n, 4))}
    for i, t in enumerate(times):
        u, v, a = traj.Evaluate(t)
        ideal['pos'][i] = u
        ideal['vel'][i] = v
        ideal['acc'][i] = a

    # Adjust vertical position to match initial condition
    z0 = data['verticalDisp'][0, 3]
    ideal['pos'][:, 0] += z0

    return times, actual, ideal


def plot_all(times, actual, ideal):
    """Create comprehensive 3x4 plot grid."""
    labels = ['Vertical', 'Theta1', 'Theta2', 'Theta3']
    units = ['m', 'rad', 'rad', 'rad']
    vel_units = ['m/s', 'rad/s', 'rad/s', 'rad/s']
    acc_units = ['m/s²', 'rad/s²', 'rad/s²', 'rad/s²']

    fig, axs = plt.subplots(3, 4, figsize=(20, 15))

    # Position plots
    for i in range(4):
        axs[0, i].plot(times, actual['pos'][:, i], 'b-', label='Actual')
        axs[0, i].plot(times, ideal['pos'][:, i], 'r--', label='Ideal')
        axs[0, i].set_title(f'{labels[i]} Position ({units[i]})')
        axs[0, i].legend()
        axs[0, i].grid(True)

    # Velocity plots
    for i in range(4):
        axs[1, i].plot(times, actual['vel'][:, i], 'b-', label='Actual')
        axs[1, i].plot(times, ideal['vel'][:, i], 'r--', label='Ideal')
        axs[1, i].set_title(f'{labels[i]} Velocity ({vel_units[i]})')
        axs[1, i].legend()
        axs[1, i].grid(True)

    # Acceleration plots
    for i in range(4):
        axs[2, i].plot(times, actual['acc_calc'][:, i], 'b-', label='Calculated')
        axs[2, i].plot(times, ideal['acc'][:, i], 'r--', label='Ideal')
        axs[2, i].set_title(f'{labels[i]} Acceleration ({acc_units[i]})')
        axs[2, i].legend()
        axs[2, i].grid(True)

    plt.tight_layout(pad=2.0)
    plt.savefig('all_sensors_data_with_ideal.png', dpi=300)
    plt.close()


def plot_errors(times, actual, ideal):
    """Create error plots."""
    fig, axs = plt.subplots(3, 3, figsize=(15, 15))

    for i, var in enumerate(['pos', 'vel', 'acc']):
        for j in range(3):
            err = ideal[var][:, j] - actual[var if var != 'acc' else 'acc_calc'][:, j]
            axs[i, j].plot(times, err)
            axs[i, j].set_title(
                f'{["Vertical", "Theta1", "Theta2"][j]} {["Position", "Velocity", "Acceleration"][i]} Error')
            axs[i, j].set_ylabel('Error')
            if i == 2:
                axs[i, j].set_xlabel('Time (s)')
            axs[i, j].grid(True)

    plt.tight_layout()
    plt.savefig('all_errors.png', dpi=300)
    plt.close()


def plot_torques(torque_values, tEnd):
    """Plot joint torques/forces with proper time scaling."""
    if not torque_values:
        print("No torque data available!")
        return

    torques = np.array(torque_values)
    n_steps = len(torques)

    # Create proper time vector based on simulation duration
    torque_times = np.linspace(0, tEnd, n_steps)

    # Separate cylinder force plot (Pz joint)
    plt.figure(figsize=(12, 6))
    plt.plot(torque_times, torques[:, 0], 'b-', linewidth=2.5, label='Cylinder Force (Pz)')
    plt.title('Cylinder Actuation Force', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Force (N)', fontsize=12)
    plt.legend(fontsize=12)
    plt.grid(True, alpha=0.7)
    plt.tight_layout()
    plt.savefig('cylinder_force.png', dpi=300)
    plt.close()

    # Combined torque plot
    plt.figure(figsize=(12, 8))
    plt.plot(torque_times, torques[:, 0], label='Cylinder (Pz)')
    plt.plot(torque_times, torques[:, 1], label='Link 1 (Rz)')
    plt.plot(torque_times, torques[:, 2], label='Link 2 (Rz)')
    plt.plot(torque_times, torques[:, 3], label='Link 3 (Rz)')

    plt.title('Joint Actuation Forces and Torques', fontsize=14)
    plt.xlabel('Time (s)', fontsize=12)
    plt.ylabel('Force/Torque (N or N·m)', fontsize=12)
    plt.legend()
    plt.grid(True, alpha=0.7)
    plt.tight_layout()
    plt.savefig('all_torques.png', dpi=300)
    plt.close()


def save_debug_data(torque_values, output_dir):
    """Save torque data for debugging."""
    with open(os.path.join(output_dir, "Torques.txt"), "w") as f:
        for tau in torque_values:
            f.write(f"{tau}\n")


# Main execution
if __name__ == "__main__":
    robot = create_robot()
    traj = create_trajectory()
    SC, mbs, sim_settings, torque_values, sensors = setup_simulation(robot, traj)
    run_simulation(SC, mbs, sim_settings)
    data = get_sensor_data(mbs, sensors)
    times, actual, ideal = process_data(data, traj)
    plot_all(times, actual, ideal)
    plot_errors(times, actual, ideal)
    plot_torques(torque_values, tEnd)  # Pass tEnd for proper time scaling
    save_debug_data(torque_values, output_dir)
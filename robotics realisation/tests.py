from src.trajectory_opt import *
from scipy.interpolate import Akima1DInterpolator
import autograd.numpy as np

jnt1_spl = sc.interpolate.splrep(theta, jnt_traj[:, 0])
jnt2_spl = sc.interpolate.splrep(theta, jnt_traj[:, 1])

def q1_of_theta(th):   
    return sc.interpolate.splev(th, jnt1_spl, der=0)

def dq1_dtheta(th):    
    return sc.interpolate.splev(th, jnt1_spl, der=1)

def d2q1_dtheta2(th):  
    return sc.interpolate.splev(th, jnt1_spl, der=2)

def q2_of_theta(th):   
    return sc.interpolate.splev(th, jnt2_spl, der=0)

def dq2_dtheta(th):    
    return sc.interpolate.splev(th, jnt2_spl, der=1)

def d2q2_dtheta2(th):  
    return sc.interpolate.splev(th, jnt2_spl, der=2)


theta_of_t = Akima1DInterpolator(time, theta)              # θ(t)
thetadot_of_t = Akima1DInterpolator(time, velocity_profile)   # θ̇(t)
thetaddot_of_t = Akima1DInterpolator(time, (np.gradient(velocity_profile, time, edge_order=2))) # θ̈(t)

t_min, t_max = float(time[0]), float(time[-1])


def q_of_t(t):
    """Generalized coordinates q(t)"""
    t_clamped = float(np.clip(t, t_min, t_max))
    th = float(theta_of_t(t_clamped))

    q1 = q1_of_theta(th)
    q2 = q2_of_theta(th)

    qz = 0.0      
    q3 = -0.5 * q2

    return np.array([qz, q1, q2, q3], dtype=float)


def qd_of_t(t):
    """Generalized velocities q̇(t)"""
    t_clamped = float(np.clip(t, t_min, t_max))
    th  = float(theta_of_t(t_clamped))
    thd = float(thetadot_of_t(t_clamped))

    q1d = dq1_dtheta(th) * thd
    q2d = dq2_dtheta(th) * thd

    qzd = 0.0
    q3d = 0

    return np.array([qzd, q1d, q2d, q3d], dtype=float)


def qdd_of_t(t):
    """Generalized accelerations q̈(t)"""
    t_clamped = float(np.clip(t, t_min, t_max))
    th   = float(theta_of_t(t_clamped))
    thd  = float(thetadot_of_t(t_clamped))
    thdd = float(thetaddot_of_t(t_clamped))

    q1dd = d2q1_dtheta2(th) * thd**2 + dq1_dtheta(th) * thdd
    q2dd = d2q2_dtheta2(th) * thd**2 + dq2_dtheta(th) * thdd

    qzdd = 0.0
    q3dd = 0

    return np.array([qzdd, q1dd, q2dd, q3dd], dtype=float)


class RobotTrajectoryObj:
    def __init__(self):
        self.t0 = t_min
        self.t_end = t_max

    def Evaluate(self, t):
        """Return (q, qd, qdd) for time t"""
        return q_of_t(t), qd_of_t(t), qdd_of_t(t)

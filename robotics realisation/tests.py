from src.trajectory_opt import *
from scipy.interpolate import Akima1DInterpolator

theta_of_t     = Akima1DInterpolator(time, theta)              # θ(t)
thetadot_of_t  = Akima1DInterpolator(time, velocity_profile)   # θ̇(t)
thetaddot_arr  = np.gradient(velocity_profile, time, edge_order=2)
thetaddot_of_t = Akima1DInterpolator(time, thetaddot_arr)      # θ̈(t)

# helpers for q(θ)
def q1_of_theta(th):    return sc.interpolate.splev(th, jnt1_spl, der=0)
def dq1_dtheta(th):     return sc.interpolate.splev(th, jnt1_spl, der=1)
def d2q1_dtheta2(th):   return sc.interpolate.splev(th, jnt1_spl, der=2)
def q2_of_theta(th):    return sc.interpolate.splev(th, jnt2_spl, der=0)
def dq2_dtheta(th):     return sc.interpolate.splev(th, jnt2_spl, der=1)
def d2q2_dtheta2(th):   return sc.interpolate.splev(th, jnt2_spl, der=2)

t_min, t_max = float(time[0]), float(time[-1])

def q_of_t(t):
    t_clamped = float(np.clip(t, t_min, t_max))
    th = float(theta_of_t(t_clamped))
    q1 = float(q1_of_theta(th))
    q2 = float(q2_of_theta(th))
    qz = 0.0
    q3 = 0.5 * q2
    return np.array([qz, q1, q2, q3], dtype=float)

def qd_of_t(t):
    t_clamped = float(np.clip(t, t_min, t_max))
    th  = float(theta_of_t(t_clamped))
    thd = float(thetadot_of_t(t_clamped))
    q1d = float(dq1_dtheta(th)) * thd
    q2d = float(dq2_dtheta(th)) * thd
    qzd = 0.0
    q3d = 0.5 * q2d
    return np.array([qzd, q1d, q2d, q3d], dtype=float)

def qdd_of_t(t):
    t_clamped = float(np.clip(t, t_min, t_max))
    th   = float(theta_of_t(t_clamped))
    thd  = float(thetadot_of_t(t_clamped))
    thdd = float(thetaddot_of_t(t_clamped))
    q1dd = float(d2q1_dtheta2(th)) * thd**2 + float(dq1_dtheta(th)) * thdd
    q2dd = float(d2q2_dtheta2(th)) * thd**2 + float(dq2_dtheta(th)) * thdd
    qzdd = 0.0
    q3dd = 0.5 * q2dd
    return np.array([qzdd, q1dd, q2dd, q3dd], dtype=float)

class RobotTrajectoryObj:
    def __init__(self):
        self.t0 = t_min
        self.t_end = t_max

    def Evaluate(self, t):
        """Return (q, qd, qdd) for time t"""
        return q_of_t(t), qd_of_t(t), qdd_of_t(t)

robotTrajectory = RobotTrajectoryObj()


try:
    _ = theta_from_time
except NameError:
    theta_from_time = theta_of_t
    veloc_from_time = thetadot_of_t
tt = time.copy()
import autograd.numpy as np  # Thinly-wrapped numpy
import matplotlib.pyplot as plt
import scipy as sc
from scipy.linalg import solve
# import sympy as sp
# from sympy import Derivative, Function, symbols, Subs
from scipy.interpolate import CubicSpline
# from sympy.abc import x, y
from scipy.differentiate import jacobian as sc_jac
# l1, l2 = symbols('l1 l2')
# Derivative(l1*sp.cos(x) + l1*sp.cos(x +y) + l2*sp.cos(x + 0.5*y),x, evaluate=True)
# Derivative(l1*sp.sin(x) + l1*sp.sin(x +y) + l2*sp.sin(x + 0.5*y),x, evaluate=True)
# Derivative(l1*sp.cos(x) + l1*sp.cos(x +y) + l2*sp.cos(x + 0.5*y),y, evaluate=True)
# Derivative(l1*sp.sin(x) + l1*sp.sin(x +y) + l2*sp.sin(x + 0.5*y),y, evaluate=True)
import nurbspy as nrb



def forward_kin(jnt, params):
    l1 = params.first_link_length
    l2 = params.second_link_length
    return np.array([
        l1*np.cos(jnt[0]) + l1*np.cos(jnt[0]+jnt[1]) + l2*np.cos(jnt[0]+0.5*jnt[1]),
        l1*np.sin(jnt[0]) + l1*np.sin(jnt[0]+jnt[1]) + l2*np.sin(jnt[0]+0.5*jnt[1])
    ])  
def jacobian(jnt, params):
    l1 = params.first_link_length
    l2 = params.second_link_length
    return np.array([
        [
            - l1 * np.sin(jnt[0]) - l1 * np.sin(jnt[0] + jnt[1]) - l2 * np.sin(jnt[0] + 0.5 * jnt[1]),
            - l1 * np.sin(jnt[0] + jnt[1]) - 0.5 * l2 * np.sin(jnt[0] + 0.5 * jnt[1])
        ], 
        [
            l1*np.cos(jnt[0]) + l1*np.cos(jnt[0] + jnt[1]) + l2*np.cos(jnt[0] + 0.5*jnt[1]),
            l1*np.cos(jnt[0] + jnt[1]) + 0.5*l2*np.cos(jnt[0] + 0.5*jnt[1])
        ]
    ])


class Params:
    def __init__(self):
        self.first_link_length = 0.205
        self.second_link_length = 0.350


pars = Params();
fk = lambda jnt: forward_kin(jnt, pars)
J1 = lambda jnt: jacobian(jnt, pars)
def inv_kin(axes, forkin, jac, init_guess):
    jnt = init_guess
    for i in range(100):
        jnt = jnt - sc.linalg.solve(jac(jnt), (forkin(jnt)-axes))
        if np.linalg.norm(forkin(jnt) - axes) < 1e-6:
            return jnt
    return np.array([np.nan, np.nan])


jnt0 = np.array([0.01, -0.02])

J1(jnt0)


# Define the array of control points
t = 0.7

axs0 = fk(jnt0)
axs1 = np.array([axs0[1], axs0[0]])
P = np.array([
    axs0,
    [t, 0.0],
    [0.0, t],
    axs1,
]).T



# Create and plot the Bezier curve
bezierCurve = nrb.NurbsCurve(control_points=P)
bezierCurve.plot()

plt.show()
N = 2000
theta_to_axis = lambda th: np.array([
    axs1[0] * th + axs0[0] * (1 - th), 
    axs1[1] * th + axs0[1] * (1 - th), 
]).T
theta = np.hstack(
    (
        np.linspace(0, 1, N),
        # np.linspace(0.05, 1, N)[1:],
    )
)


axs_traj = bezierCurve.get_value(theta).T
# plt.plot(axs_traj[:,0], axs_traj[:,1], '-o'); plt.show()
jnt_traj = np.empty(axs_traj.shape)
jnt_traj[0,:] = jnt0
for i in range(1,N):
    jnt_traj[i,:] = inv_kin(axs_traj[i,:], fk, J1, jnt_traj[i-1,:])

axs_traj_test = np.empty(axs_traj.shape)
for i in range(0,N):
    axs_traj_test[i,:] = fk(jnt_traj[i,:])
plt.plot(theta, jnt_traj[:,0], '-')
plt.plot(theta, jnt_traj[:,1], '-')
plt.show()
# Compute derivative of joint trajectory using 4th order schemes with vectorized operations
# Calculate step size based on theta spacing
h = theta[1]-theta[0]  # theta goes from 0 to 1 with N points

jnt1_spl = sc.interpolate.splrep(theta, jnt_traj[:,0])
jnt2_spl = sc.interpolate.splrep(theta, jnt_traj[:,1])




jnt_traj_derivative = np.array([sc.interpolate.splev(theta, jnt1_spl, der=1), sc.interpolate.splev(theta, jnt2_spl, der=1)]).T


q1_lim_vel = 2000 / 60 * 2 * np.pi
q2_lim_vel = 2000 / 60 * 2 * np.pi
q1_lim_vel *= 0.01
q2_lim_vel *= 0.01
def d_th_dt_lim(th): 
    return np.array([
        q1_lim_vel / sc.interpolate.splev(th, jnt1_spl, der=1),
        q2_lim_vel / sc.interpolate.splev(th, jnt2_spl, der=1)
    ])
limits = d_th_dt_lim(theta)
plt.plot(theta, limits[0,:], '-')
plt.plot(theta, -limits[0,:], '-')
plt.plot(theta, limits[1,:], '-')
plt.plot(theta, -limits[1,:], '-')
plt.show()

lims = np.min(np.abs(limits.T), axis=1)
indices = np.r_[0:len(theta):50, len(theta)-1]
plt.plot(theta[indices], lims[indices], '.')
plt.plot(theta[indices], -lims[indices],'.')
plt.show()
velocity_profile_knots = lims[indices]
velocity_profile_knots[0] = 0
velocity_profile_knots[-1] = 0
plt.plot(theta[indices], velocity_profile_knots, '.')
plt.plot(theta, lims)
velocity_profile = sc.interpolate.Akima1DInterpolator(theta[indices], velocity_profile_knots, method="makima")(theta)
for i in np.linspace(0.6, 0.95, 4):
    plt.plot(theta, velocity_profile*i)
plt.show()



time = np.zeros(theta.shape)
time[1:-1] = np.cumsum(1/velocity_profile[1:-1])*h
coefficients = np.polyfit(theta[-4:-1], time[-4:-1], 2)
polynomial = np.poly1d(coefficients)
time[-1] = polynomial(theta[-1])
plt.plot(time, theta)
plt.plot(time, velocity_profile)

tt = np.linspace(0, time[-1], 1000)
theta_from_time = sc.interpolate.Akima1DInterpolator(time, theta, method="makima")(tt)
veloc_from_time = sc.interpolate.Akima1DInterpolator(time, velocity_profile, method="makima")(tt)
plt.plot(tt, sc.interpolate.splev(theta_from_time, jnt1_spl))
plt.plot(tt, sc.interpolate.splev(theta_from_time, jnt2_spl))

# plt.plot(tt, theta_from_time, 'r')
# plt.plot(tt, veloc_from_time, 'b')
plt.show()

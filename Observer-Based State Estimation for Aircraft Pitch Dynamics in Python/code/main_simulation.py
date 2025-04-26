# -*- coding: utf-8 -*-
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.signal import place_poles
# Constants
g = 9.81          # gravity (m/s^2)
D = 0.1           # vertical drag coefficient (1/s)
a = 0.5           # pitch damping coefficient (1/s)
b = 2.0           # elevator control effectiveness (1/s^2)

# System matrices
A = np.array([[0, 1, 0, 0],
              [0, -a, 0, 0],
              [0, 0, 0, 1],
              [0, 0, 0, -D]])

B = np.array([[0],
              [b],
              [0],
              [0]])

C = np.array([[1, 0, 0, 0],
              [0, 0, 0, 1]])
# Observer Gain Matrix


O = np.vstack([
    C,
    C @ A,
    C @ A @ A,
    C @ A @ A @ A
])

rank_O = np.linalg.matrix_rank(O)
print("Observability Matrix Rank:", rank_O)

##
system_poles = np.linalg.eigvals(A)
print("System poles:", system_poles)

L = np.array([[5, 0],
              [15, 0],
              [0, 8],
              [0, 12]])

def aircraft_dynamics(t, x, u_func):
    theta, q, v, h = x  
    u = u_func(t)       # Evaluate control input at current time
    # Dynamics equations
    dtheta_dt = q
    dq_dt = -a * q + b * u
    dv_dt = g * np.sin(theta) - D * v
    dh_dt = v

    dxdt = [dtheta_dt, dq_dt, dv_dt, dh_dt]
    return dxdt
#

def observer_dynamics(t, x_hat, u_func, y_func, A, B, C, L):
    u = u_func(t)           
    y_meas = y_func(t)        # Measured outputs at time t
    y_hat = C @ x_hat         # Estimated outputs based on estimated states

    dxhat_dt = A @ x_hat + B.flatten() * u + L @ (y_meas - y_hat)
    return dxhat_dt
#
def elevator_input(t):
    #Small elevator pulse for first few seconds.
    if 1 <= t <= 3:
        return 5.0 * np.pi/180   # 5 degrees deflection in radians
    else:
        return 0.0
    
    
def measurement_function(t, sol):
 
    # Interpolate the solution to get the states at time t
    x_true = np.array([np.interp(t, sol.t, sol.y[i, :]) for i in range(4)])
    
    # Measurements
    y_meas = C @ x_true
    return y_meas

#
# Initial conditions: [theta, q, v, h]
x0 = [0.0, 0.0, 0.0, 1000.0]   # starting flat, stationary, at 1000 m altitude
# Initial estimated conditions (observer)
x_hat0 = [0.0, 0.0, 0.0, 1000.0]  # Start with same altitude but 0 guess for everything else

# Simulation time
t_start = 0
t_end = 20
dt = 0.01
t_span = (t_start, t_end)
t_eval = np.arange(t_start, t_end, dt)

# Solve ODE
sol = solve_ivp(lambda t, x: aircraft_dynamics(t, x, elevator_input),t_span, x0, t_eval=t_eval)

# Solve Observer ODE
sol_hat = solve_ivp(lambda t, x_hat: observer_dynamics(t, x_hat, elevator_input, 
                                                        lambda t_: measurement_function(t_, sol),
                                                        A, B, C, L), t_span, x_hat0, t_eval=t_eval)


# Plotting
theta = sol.y[0, :]
q = sol.y[1, :]
v = sol.y[2, :]
h = sol.y[3, :]

plt.figure(figsize=(12,8))
plt.subplot(2,2,1)
plt.plot(sol.t, theta * 180/np.pi)
plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle θ (deg)')
plt.grid(True)

plt.subplot(2,2,2)
plt.plot(sol.t, q * 180/np.pi)
plt.xlabel('Time (s)')
plt.ylabel('Pitch Rate q (deg/s)')
plt.grid(True)

plt.subplot(2,2,3)
plt.plot(sol.t, v)
plt.xlabel('Time (s)')
plt.ylabel('Vertical Speed v (m/s)')
plt.grid(True)

plt.subplot(2,2,4)
plt.plot(sol.t, h)
plt.xlabel('Time (s)')
plt.ylabel('Altitude h (m)')
plt.grid(True)
plt.tight_layout()
plt.show()

# Extract states
theta_real = sol.y[0, :]
q_real = sol.y[1, :]
v_real = sol.y[2, :]
h_real = sol.y[3, :]

theta_hat = sol_hat.y[0, :]
q_hat = sol_hat.y[1, :]
v_hat = sol_hat.y[2, :]
h_hat = sol_hat.y[3, :]

# Plot estimated vs real
plt.figure(figsize=(14,10))

plt.subplot(2,2,1)
plt.plot(sol.t, theta_real * 180/np.pi, label='True θ')
plt.plot(sol_hat.t, theta_hat * 180/np.pi, '--', label='Estimated θ')
plt.xlabel('Time (s)')
plt.ylabel('Pitch Angle (deg)')
plt.legend()
plt.grid(True)

plt.subplot(2,2,2)
plt.plot(sol.t, q_real * 180/np.pi, label='True q')
plt.plot(sol_hat.t, q_hat * 180/np.pi, '--', label='Estimated q')
plt.xlabel('Time (s)')
plt.ylabel('Pitch Rate (deg/s)')
plt.legend()
plt.grid(True)

plt.subplot(2,2,3)
plt.plot(sol.t, v_real, label='True v')
plt.plot(sol_hat.t, v_hat, '--', label='Estimated v')
plt.xlabel('Time (s)')
plt.ylabel('Vertical Speed (m/s)')
plt.legend()
plt.grid(True)

plt.subplot(2,2,4)
plt.plot(sol.t, h_real, label='True h')
plt.plot(sol_hat.t, h_hat, '--', label='Estimated h')
plt.xlabel('Time (s)')
plt.ylabel('Altitude (m)')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()


# Estimation Errors
theta_error = theta_real - theta_hat
q_error = q_real - q_hat
v_error = v_real - v_hat
h_error = h_real - h_hat

plt.figure(figsize=(14,8))

plt.subplot(2,2,1)
plt.plot(sol.t, theta_error * 180/np.pi)
plt.title('Pitch Angle Estimation Error (deg)')
plt.grid(True)

plt.subplot(2,2,2)
plt.plot(sol.t, q_error * 180/np.pi)
plt.title('Pitch Rate Estimation Error (deg/s)')
plt.grid(True)

plt.subplot(2,2,3)
plt.plot(sol.t, v_error)
plt.title('Vertical Velocity Estimation Error (m/s)')
plt.grid(True)

plt.subplot(2,2,4)
plt.plot(sol.t, h_error)
plt.title('Altitude Estimation Error (m)')
plt.grid(True)

plt.tight_layout()
plt.show()
















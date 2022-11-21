#    LMPC_walking is a python software implementation of some of the linear MPC
#    algorithms based presented in:
#    https://groups.csail.mit.edu/robotics-center/public_papers/Wieber15.pdf
#    Copyright (C) 2019 @ahmad gazar

#    LMPC_walking is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.

#    LMPC_walking is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.

#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <https://www.gnu.org/licenses/>.


# headers:
# -------
import numpy as np
from quadprog import solve_qp
import reference_trajectories
import cost_function
import constraints
import motion_model
import plot_utils

# cost weights in the objective function:
# ---------------------------------------
alpha       = 10**(-1)     # CoP error squared cost weight
beta        = 0*10**(-3)   # CoM position error squared cost weight
gamma       = 10**(-3)     # CoM velocity error squared cost weight

# Inverted pendulum parameters:
# ----------------------------
h           = 0.80
g           = 9.81
foot_length = 0.20
foot_width  = 0.10

# MPC Parameters:
# --------------
delta_t               = 0.1                                # sampling time interval
step_time             = 0.8                                # time needed for every step
no_steps_per_T        = int(round(step_time/delta_t))
N                     = 18                                 # preceding horizon

# walking parameters:
# ------------------
step_length           = 0.21                               # fixed step length in the xz-plane
no_desired_steps      = 20                                 # number of desired walking steps
no_planned_steps      = 2+no_desired_steps                 # planning 2 steps ahead (increase if you want to increase the horizon)
desired_walking_time  = no_desired_steps * no_steps_per_T  # number of desired walking intervals
planned_walking_time  = no_planned_steps * no_steps_per_T  # number of planned walking intervals

# CoM initial state: [x, xdot, x_ddot].T
#                    [y, ydot, y_ddot].T
# --------------------------------------
x_hat_0 = np.array([0.0, 0.0])
y_hat_0 = np.array([-0.09, 0.0])

step_width = 2*np.absolute(y_hat_0[0])

# compute CoP reference trajectory:
# --------------------------------
foot_step_0   = np.array([0.0, -0.09])    # initial foot step position in x-y

desiredFoot_steps  = reference_trajectories.manual_foot_placement(foot_step_0,
                                                step_length, no_desired_steps)
desired_Z_ref = reference_trajectories.create_CoP_trajectory(no_desired_steps,
                        desiredFoot_steps, desired_walking_time, no_steps_per_T)

#plannedFoot_steps = reference_trajectories.manual_foot_placement(foot_step_0,\
#                                                step_length, no_planned_steps)
#planned_Z_ref = reference_trajectories.create_CoP_trajectory(no_planned_steps,\
#                       plannedFoot_steps, planned_walking_time, no_steps_per_T)

# plan the last 2 steps CoP reference in the future to be the same as last step
planned_Z_ref = np.zeros((planned_walking_time, 2))
planned_Z_ref[0:desired_walking_time,:] =  desired_Z_ref
planned_Z_ref[desired_walking_time:planned_walking_time,:] = desired_Z_ref[desired_walking_time-1,:]

x_hat_k   = x_hat_0
y_hat_k   = y_hat_0
Z_ref_k   = planned_Z_ref[0:N,:]

# pre-allocate memory
X_k       = np.zeros((N,2))
Y_k       = np.zeros((N,2))
Z_x_k     = np.zeros((N))
Z_y_k     = np.zeros((N))

X_total   = np.zeros((desired_walking_time,2))
Y_total   = np.zeros((desired_walking_time,2))
Z_x_total = np.zeros((desired_walking_time))
Z_y_total = np.zeros((desired_walking_time))

T_k       = 0.0
time_k    = np.zeros((N))
[P_ps, P_vs, P_pu, P_vu] = motion_model.compute_recursive_matrices(delta_t,
                                                                   g, h, N)

# create a dictionary for data logging of horizons at every sampling time T
horizon_data = {}

# MPC loop:
# --------
for i in range(desired_walking_time):

    time_k   = np.arange(T_k, T_k + (N*delta_t), delta_t)
    #print 'T_k = ', T_k, 's\n'
    #print 'loop number = ', i, '\n'
    #print 'Z_ref_k = ', Z_ref_k

    horizon_data[i] = {}
    horizon_data[i]['zmp_reference'] = Z_ref_k
    [Q, p_k] = cost_function.compute_objective_terms(alpha, beta, gamma,
                        step_time, no_steps_per_T, N, step_length, step_width,
                        P_ps, P_pu, P_vs, P_vu, x_hat_k, y_hat_k, Z_ref_k)

    [A_zmp, b_zmp] = constraints.add_ZMP_constraints(N, foot_length, foot_width,
                                                Z_ref_k, x_hat_k, y_hat_k)
    # call quadprog solver:
    current_U = solve_qp(Q, -p_k, A_zmp.T, b_zmp)[0]

    # evaluate your recursive dynamics over the current horizon:
    [X_k, Y_k] = motion_model.compute_recursive_dynamics(P_ps, P_vs, P_pu,
                                                         P_vu, N, x_hat_k,
                                                         y_hat_k, current_U)

    # save horizon data at every optimization iteration
    horizon_data[i]['X_k']   = X_k
    horizon_data[i]['Y_k']   = Y_k
    horizon_data[i]['Z_x_k'] = Z_x_k
    horizon_data[i]['Z_y_k'] = Z_y_k
    horizon_data[i]['time_k'] = time_k

    # save CoM trajectories
    X_total[i,:]  = X_k[0,:]
    Y_total[i,:]  = Y_k[0,:]

    # save CoP trajectories
    Z_x_total[i]  = current_U[0]
    Z_y_total[i]  = current_U[N]

    # update the current state for next iteration
    x_hat_k   = X_k[0,:]
    y_hat_k   = Y_k[0,:]

    # update CoP reference trajectory for next iteration
    Z_ref_k   = planned_Z_ref[i+1:i+N+1,:]

    # update time step
    T_k = T_k + delta_t
# ------------------------------------------------------------------------------
# debugging:
# ------------------------------------------------------------------------------
    #print horizon_data[i]
    #print 'x_hat_k     = ', x_hat_k[0], '\n'
    #print 'x_hatdot_k  = ', X_k[0,1], '\n'
    #print 'x_hatddot_k = ', X_k[0,2], '\n'

    #print 'y_hat_k     = ', y_hat_k[0], '\n'
    #print 'y_hatdot_k  = ', Y_k[0,1], '\n'
    #print 'y_hatddot_k = ', Y_k[0,2], '\n'

    #print 'Z_x_k = ', Z_x_k[0], '\n'
    #print 'Z_y_k = ', Z_y_k[0], '\n'

    #print 'Z_ref_k = ',  Z_ref_k
# ------------------------------------------------------------------------------
# visualize your final trajectory:
# ------------------------------------------------------------------------------
time  = np.arange(0, round(desired_walking_time*delta_t, 2), delta_t)
min_admissible_CoP = desired_Z_ref - np.tile([foot_length/2, foot_width/2],
                    (desired_walking_time,1))
max_admissible_cop = desired_Z_ref + np.tile([foot_length/2, foot_width/2],
                    (desired_walking_time,1))

# time vs CoP and CoM in x: 'A.K.A run rabbit run !'
# -------------------------------------------------
plot_utils.plot_x(time, desired_walking_time, min_admissible_CoP,
                  max_admissible_cop, Z_x_total, X_total, desired_Z_ref)

# time VS CoP and CoM in y: 'A.K.A what goes up must go down'
# ----------------------------------------------------------
plot_utils.plot_y(time, desired_walking_time, min_admissible_CoP,
                  max_admissible_cop, Z_y_total, Y_total, desired_Z_ref)

# plot CoP, CoM in x Vs Cop, CoM in y:
# -----------------------------------
plot_utils.plot_xy(time, desired_walking_time, foot_length, foot_width,
                  desired_Z_ref, Z_x_total, Z_y_total, X_total, Y_total)

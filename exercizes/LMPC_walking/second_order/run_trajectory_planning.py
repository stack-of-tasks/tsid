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
import motion_model
import cost_function
import constraints
import plot_utils

# cost weights in the objective function:
# ---------------------------------------
alpha       = 10**(-1)   # CoP error squared cost weight
beta        = 0*10**(-3) # CoM position error squared cost weight
gamma       = 10**(-3)   # CoM velocity error squared cost weight

# Inverted pendulum parameters:
# ----------------------------
h           = 0.80   # fixed CoM height (assuming walking on a flat terrain)
g           = 9.81   # norm of the gravity vector
foot_length = 0.20   # foot size in the x-direction
foot_width  = 0.10   # foot size in the y-direciton

# MPC Parameters:
# --------------
delta_t               = 0.1                         # sampling time interval
step_time             = 0.8                         # time needed for every step
no_steps_per_T        = int(round(step_time/delta_t))

# walking parameters:
# ------------------
step_length           = 0.21                  # fixed step length in the xz-plane
no_desired_steps      = 6                     # number of desired walking steps
desired_walking_time  = no_desired_steps * no_steps_per_T  # number of desired walking intervals
N                     = desired_walking_time  # preceding horizon

# CoM initial state: [x_0, xdot_0].T
#                    [y_0, ydot_0].T
# ----------------------------------
x_0 = np.array([0.0, 0.0])
y_0 = np.array([-0.09, 0.0])

step_width = 2*np.absolute(y_0[0])

# compute CoP reference trajectory:
# --------------------------------
foot_step_0   = np.array([0.0, -0.09])   # initial foot step position in x-y

desiredFoot_steps  = reference_trajectories.manual_foot_placement(foot_step_0,
                                                step_length, no_desired_steps)
desired_Z_ref = reference_trajectories.create_CoP_trajectory(no_desired_steps,
                    desiredFoot_steps, desired_walking_time, no_steps_per_T)

# used in case you want to have terminal constraints
# -------------------------------------------------
x_terminal = np.array([desired_Z_ref[N-1, 0], 0.0])  # CoM terminal constraint in x : [x, xdot].T
y_terminal = np.array([desired_Z_ref[N-1, 1], 0.0])  # CoM terminal constraint in y : [y, ydot].T
no_terminal_constraints = 4
terminal_index = N-1

# construct your preview system: 'Go pokemon !'
# --------------------------------------------
[P_ps, P_vs, P_pu, P_vu] = motion_model.compute_recursive_matrices(delta_t, g,
                                                                   h, N)
[Q, p_k] = cost_function.compute_objective_terms(alpha, beta, gamma,
                        step_time, no_steps_per_T, N, step_length, step_width,
                        P_ps, P_pu, P_vs, P_vu, x_0, y_0, desired_Z_ref)
[A_zmp, b_zmp] = constraints.add_ZMP_constraints(N, foot_length, foot_width,
                                                desired_Z_ref, x_0, y_0)

# used in case you want to add both terminal add_ZMP_constraints
# --------------------------------------------------------------
[A_terminal, b_terminal] = constraints.add_terminal_constraints(N,
                            terminal_index, x_0, y_0, x_terminal,
                             y_terminal, P_ps, P_vs, P_pu, P_vu)
A = np.concatenate((A_terminal, A_zmp), axis = 0)
b = np.concatenate((b_terminal, b_zmp), axis = 0)

# call quadprog solver:
# --------------------
#U = solve_qp(Q, -p_k, A.T, b, no_terminal_constraints)[0]  # uncomment to solve with 4 equality terminal constraints
U = solve_qp(Q, -p_k, A_zmp.T, b_zmp)[0]                    # solve only with only CoP inequality constraints
Z_x_total = U[0:N]
Z_y_total = U[N:2*N]

# Trajectory optimization: (based on the initial state x_hat_0, y_hat_0)
# -------------------------------------------------------------------------
[X_total, Y_total] = motion_model.compute_recursive_dynamics(P_ps, P_vs, P_pu,
                                                             P_vu, N, x_0,
                                                             y_0, U)
# ------------------------------------------------------------------------------
# visualize your open-loop trajectory:
# ------------------------------------------------------------------------------
time = np.arange(0, round(desired_walking_time*delta_t, 2), delta_t)
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

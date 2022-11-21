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
import LMPC_walking.second_order.reference_trajectories as reference_trajectories
import LMPC_walking.second_order.motion_model as motion_model
import LMPC_walking.second_order.cost_function as cost_function
import LMPC_walking.second_order.constraints as constraints
import LMPC_walking.second_order.plot_utils as plot_utils
import matplotlib.pyplot as plt
from plot_utils import *
import ex_4_conf as conf
#import ex_4_long_conf as conf

# Inverted pendulum parameters:
# ----------------------------
foot_length = conf.lxn + conf.lxp   # foot size in the x-direction
foot_width  = conf.lyn + conf.lyp   # foot size in the y-direciton
nb_dt_per_step        = int(round(conf.T_step/conf.dt_mpc))
N  = conf.nb_steps * nb_dt_per_step  # number of desired walking intervals

# CoM initial state: [x_0, xdot_0].T
#                    [y_0, ydot_0].T
# ----------------------------------
x_0 = np.array([conf.foot_step_0[0], 0.0])
y_0 = np.array([conf.foot_step_0[1], 0.0])

step_width = 2*np.absolute(y_0[0])

# compute CoP reference trajectory:
# --------------------------------
foot_steps  = reference_trajectories.manual_foot_placement(conf.foot_step_0,
                                                conf.step_length, conf.nb_steps)
foot_steps[1:,0] -= conf.step_length
cop_ref = reference_trajectories.create_CoP_trajectory(conf.nb_steps,
                    foot_steps, N, nb_dt_per_step)

# used in case you want to have terminal constraints
# -------------------------------------------------
x_terminal = np.array([cop_ref[N-1, 0], 0.0])  # CoM terminal constraint in x : [x, xdot].T
y_terminal = np.array([cop_ref[N-1, 1], 0.0])  # CoM terminal constraint in y : [y, ydot].T
nb_terminal_constraints = 4
terminal_index = N-1

# construct your preview system: 'Go pokemon !'
# --------------------------------------------
[P_ps, P_vs, P_pu, P_vu] = motion_model.compute_recursive_matrices(conf.dt_mpc, conf.g,
                                                                   conf.h, N)
[Q, p_k] = cost_function.compute_objective_terms(conf.alpha, conf.beta, conf.gamma,
                        conf.T_step, nb_dt_per_step, N, conf.step_length, step_width,
                        P_ps, P_pu, P_vs, P_vu, x_0, y_0, cop_ref)
[A_zmp, b_zmp] = constraints.add_ZMP_constraints(N, foot_length, foot_width,
                                                cop_ref, x_0, y_0)

# used in case you want to add both terminal add_ZMP_constraints
# --------------------------------------------------------------
[A_terminal, b_terminal] = constraints.add_terminal_constraints(N,
                            terminal_index, x_0, y_0, x_terminal,
                             y_terminal, P_ps, P_vs, P_pu, P_vu)
A = np.concatenate((A_terminal, A_zmp), axis = 0)
b = np.concatenate((b_terminal, b_zmp), axis = 0)

# call quadprog solver:
# --------------------
U = solve_qp(Q, -p_k, A.T, b, nb_terminal_constraints)[0]  # uncomment to solve with 4 equality terminal constraints
cop_x = U[0:N]
cop_y = U[N:2*N]

# Trajectory optimization: (based on the initial state x_hat_0, y_hat_0)
# -------------------------------------------------------------------------
[com_state_x, com_state_y] = motion_model.compute_recursive_dynamics(P_ps, P_vs, P_pu,
                                                             P_vu, N, x_0,
                                                             y_0, U)

# ------------------------------------------------------------------------------
# visualize your open-loop trajectory:
# ------------------------------------------------------------------------------
time = np.arange(0, round(N*conf.dt_mpc, 2), conf.dt_mpc)
min_admissible_CoP = cop_ref - np.tile([foot_length/2, foot_width/2], (N,1))
max_admissible_cop = cop_ref + np.tile([foot_length/2, foot_width/2], (N,1))

# time vs CoP and CoM in x: 'A.K.A run rabbit run !'
# -------------------------------------------------
plot_utils.plot_x(time, N, min_admissible_CoP, max_admissible_cop,
                  cop_x, com_state_x, cop_ref)

# time VS CoP and CoM in y: 'A.K.A what goes up must go down'
# ----------------------------------------------------------
plot_utils.plot_y(time, N, min_admissible_CoP, max_admissible_cop,
                  cop_y, com_state_y, cop_ref)

# plot CoP, CoM in x Vs Cop, CoM in y:
# -----------------------------------
plot_utils.plot_xy(time, N, foot_length, foot_width, cop_ref, cop_x, cop_y, com_state_x, com_state_y)
import matplotlib.pyplot as plt
plt.gca().set_xlim([cop_ref[0,0]-0.2, cop_ref[-1,0]+0.2])
plt.gca().set_ylim([cop_ref[0,1]-0.2, cop_ref[-1,1]+0.2])

com_state_x = np.vstack((x_0, com_state_x))
com_state_y = np.vstack((y_0, com_state_y))
np.savez(conf.DATA_FILE_LIPM, 
         com_state_x=com_state_x, com_state_y=com_state_y, cop_ref=cop_ref, 
         cop_x=cop_x, cop_y=cop_y, foot_steps=foot_steps)

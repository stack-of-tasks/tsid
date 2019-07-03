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
from LMPC_walking.second_order.motion_model import discrete_LIP_dynamics
import matplotlib.pyplot as plt
import ex_4_conf as conf

# Inverted pendulum parameters:
# ----------------------------
foot_length = conf.lxn + conf.lxp   # foot size in the x-direction
foot_width  = conf.lyn + conf.lyp   # foot size in the y-direciton
nb_steps_per_T        = int(round(conf.T_step/conf.dt_mpc))

# walking parameters:
# ------------------
N  = conf.nb_steps * nb_steps_per_T  # number of desired walking intervals

# CoM initial state: [x_0, xdot_0].T
#                    [y_0, ydot_0].T
# ----------------------------------
x_0 = np.array([conf.foot_step_0[0], 0.0])
y_0 = np.array([conf.foot_step_0[1], 0.0])

step_width = 2*np.absolute(y_0[0])

# compute CoP reference trajectory:
# --------------------------------
desiredFoot_steps  = reference_trajectories.manual_foot_placement(conf.foot_step_0,
                                                conf.step_length, conf.nb_steps)
desiredFoot_steps[1:,0] -= conf.step_length
desired_Z_ref = reference_trajectories.create_CoP_trajectory(conf.nb_steps,
                    desiredFoot_steps, N, nb_steps_per_T)

# used in case you want to have terminal constraints
# -------------------------------------------------
x_terminal = np.array([desired_Z_ref[N-1, 0], 0.0])  # CoM terminal constraint in x : [x, xdot].T
y_terminal = np.array([desired_Z_ref[N-1, 1], 0.0])  # CoM terminal constraint in y : [y, ydot].T
nb_terminal_constraints = 4
terminal_index = N-1

# construct your preview system: 'Go pokemon !'
# --------------------------------------------
[P_ps, P_vs, P_pu, P_vu] = motion_model.compute_recursive_matrices(conf.dt_mpc, conf.g,
                                                                   conf.h, N)
[Q, p_k] = cost_function.compute_objective_terms(conf.alpha, conf.beta, conf.gamma,
                        conf.T_step, nb_steps_per_T, N, conf.step_length, step_width,
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
U = solve_qp(Q, -p_k, A.T, b, nb_terminal_constraints)[0]  # uncomment to solve with 4 equality terminal constraints
#U = solve_qp(Q, -p_k, A_zmp.T, b_zmp)[0]                    # solve only with only CoP inequality constraints
Z_x_total = U[0:N]
Z_y_total = U[N:2*N]

# Trajectory optimization: (based on the initial state x_hat_0, y_hat_0)
# -------------------------------------------------------------------------
[X_total, Y_total] = motion_model.compute_recursive_dynamics(P_ps, P_vs, P_pu,
                                                             P_vu, N, x_0,
                                                             y_0, U)
X_total = np.vstack((x_0, X_total))
Y_total = np.vstack((y_0, Y_total))

# ------------------------------------------------------------------------------
# visualize your open-loop trajectory:
# ------------------------------------------------------------------------------
time = np.arange(0, round(N*conf.dt_mpc, 2), conf.dt_mpc)
min_admissible_CoP = desired_Z_ref - np.tile([foot_length/2, foot_width/2],
                     (N,1))
max_admissible_cop = desired_Z_ref + np.tile([foot_length/2, foot_width/2],
                     (N,1))

# time vs CoP and CoM in x: 'A.K.A run rabbit run !'
# -------------------------------------------------
#plot_utils.plot_x(time, N, min_admissible_CoP,
#                  max_admissible_cop, Z_x_total, X_total, desired_Z_ref)
#
## time VS CoP and CoM in y: 'A.K.A what goes up must go down'
## ----------------------------------------------------------
#plot_utils.plot_y(time, N, min_admissible_CoP,
#                  max_admissible_cop, Z_y_total, Y_total, desired_Z_ref)
#
## plot CoP, CoM in x Vs Cop, CoM in y:
## -----------------------------------
#plot_utils.plot_xy(time, N, foot_length, foot_width,
#                   desired_Z_ref, Z_x_total, Z_y_total, X_total, Y_total)

dt_ctrl = conf.dt
N_ctrl = int((N*conf.dt_mpc)/dt_ctrl)
(A,B) = discrete_LIP_dynamics(dt_ctrl, conf.g, conf.h)
com  = np.empty((3,N_ctrl+1))*np.nan
dcom = np.zeros((3,N_ctrl+1))
ddcom = np.zeros((3,N_ctrl+1))
cop = np.empty((2,N_ctrl+1))*np.nan
#foot_steps = np.empty((2,N_ctrl+1))*np.nan
contact_phase = (N_ctrl+1)*['right']
com[2,:] = conf.h

N_inner = int(N_ctrl/N)
for i in range(N):
    com[0,i*N_inner] = X_total[i,0]
    com[1,i*N_inner] = Y_total[i,0]
    dcom[0,i*N_inner] = X_total[i,1]
    dcom[1,i*N_inner] = Y_total[i,1]
    if(i>0): 
        if np.linalg.norm(desired_Z_ref[i,:] - desired_Z_ref[i-1,:]) < 1e-10:
            contact_phase[i*N_inner] = contact_phase[i*N_inner-1]
        else:
            if contact_phase[(i-1)*N_inner]=='right':
                contact_phase[i*N_inner] = 'left'
            elif contact_phase[(i-1)*N_inner]=='left':
                contact_phase[i*N_inner] = 'right'
                
    for j in range(N_inner):
        ii = i*N_inner + j
        (A,B) = discrete_LIP_dynamics((j+1)*dt_ctrl, conf.g, conf.h)
#        foot_steps[:,ii] = desired_Z_ref[i,:].T
        cop[0,ii] = Z_x_total[i]
        cop[1,ii] = Z_y_total[i]
        x_next = A.dot(X_total[i,:]) + B.dot(cop[0,ii])
        y_next = A.dot(Y_total[i,:]) + B.dot(cop[1,ii])
        com[0,ii+1] = x_next[0]
        com[1,ii+1] = y_next[0]
        dcom[0,ii+1] = x_next[1]
        dcom[1,ii+1] = y_next[1]
        ddcom[:2,ii] = conf.g/conf.h * (com[:2,ii] - cop[:,ii])
        
        if(j>0): contact_phase[ii] = contact_phase[ii-1]

   
# Generate foot trajectories using polynomials with following constraints:
# x(0)=x0, x(T)=x1, dx(0)=dx(T)=0
# x(t) = a + b t + c t^2 + d t^3
# x(0) = a = x0
# dx(0) = b = 0
# dx(T) = 2 c T + 3 d T^2 = 0 => c = -3 d T^2 / (2 T) = -(3/2) d T
# x(T) = x0 + c T^2 + d T^3 = x1
#        x0 -(3/2) d T^3 + d T^3 = x1
#        -0.5 d T^3 = x1 - x0
#        d = 2 (x0-x1) / T^3
# c = -(3/2) T 2 (x0-x1) / (T^3) = 3 (x1-x0) / T^2
def compute_polynomial_traj(x0, x1, T, dt):
    a = x0
    b = np.zeros_like(x0)
    c = 3*(x1-x0) / (T**2)
    d = 2*(x0-x1) / (T**3)
    N = int(T/dt)
    n = x0.shape[0]
    x = np.zeros((n,N))
    dx = np.zeros((n,N))
    ddx = np.zeros((n,N))
    for i in range(N):
        t = i*dt
        x[:,i]   = a + b*t + c*t**2 + d*t**3
        dx[:,i]  = b + 2*c*t + 3*d*t**2
        ddx[:,i] = 2*c + 6*d*t
    return x, dx, ddx
    
foot_steps = desiredFoot_steps[::2,:]
#x = np.zeros((3,N_ctrl+1))
#dx = np.zeros((3,N_ctrl+1))
#ddx = np.zeros((3,N_ctrl+1))
#N_step = int(step_time/dt_ctrl)
#for s in range(foot_steps.shape[0]):
#    x[0, s*2*N_step       :s*2*N_step+N_step] = foot_steps[s,0]
#    x[1, s*2*N_step       :s*2*N_step+N_step] = foot_steps[s,1]
#    x[:2, s*2*N_step+N_step:(s+1)*2*N_step]    = compute_polynomial_traj(foot_steps[s,:], foot_steps[s+1,:], 
#                                                                         step_time, dt)
        
        



#np.savez('3_step_walking_traj', x=X_total, y=Y_total, z=desired_Z_ref)
np.savez(conf.DATA_FILE, com=com, dcom=dcom, ddcom=ddcom, 
         contact_phase=contact_phase, foot_steps=foot_steps)

time_ctrl = np.arange(0, round(N_ctrl*dt_ctrl, 2), dt_ctrl)

for i in range(2):
    plt.figure()
    plt.plot(time_ctrl, cop[i,:-1], label='CoP')
#    plt.plot(time_ctrl, foot_steps[i,:-1], label='Foot step')
    plt.plot(time_ctrl, com[i,:-1], 'g', label='CoM')
    if i==0: plt.plot(time, X_total[:-1,0], ':', label='CoM TO')
    else:    plt.plot(time, Y_total[:-1,0], ':', label='CoM TO')
    plt.legend()

for i in range(2):
    plt.figure()
    plt.plot(time_ctrl, dcom[i,:-1], label='CoM vel')
    vel_fd = (com[i,1:] - com[i,:-1]) / dt_ctrl
    plt.plot(time_ctrl, vel_fd, ':', label='CoM vel fin-diff')
    plt.legend()
    
for i in range(2):
    plt.figure()
    plt.plot(time_ctrl, ddcom[i,:-1], label='CoM acc')
    acc_fd = (dcom[i,1:] - dcom[i,:-1]) / dt_ctrl
    plt.plot(time_ctrl, acc_fd, ':', label='CoM acc fin-diff')
    plt.legend()
    
#plot_utils.plot_xy(time_ctrl, N_ctrl, foot_length, foot_width,
#                   foot_steps.T, cop[0,:], cop[1,:], 
#                   com[0,:].reshape((N_ctrl+1,1)), 
#                   com[1,:].reshape((N_ctrl+1,1)))
#plt.plot( X_total[:,0], Y_total[:,0], 'r* ', markersize=15,)

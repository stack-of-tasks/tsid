#    LMPC_walking is a python software implementation of some of the linear MPC
#    algorithms based presented in:
#    https://groups.csail.mit.edu/robotics-center/public_papers/Wieber15.pdf
#
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
from PIL import Image
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec

# Description:
# -----------
# this function compute the canonical quadratic objective term Q (hessian)
# and the linear objective term p.T (gradient) of this cost function:
# min zx_k, zy_k
#    beta/2||x-x_r||^2 + gamma/2||x_dot-x_dot_r||^2 + alpha ||z_x-zx_r||^2
#  + beta/2||y-y_r||^2 + gamma/2||y_dot-y_dot_r||^2 + alpha ||z_y-zy_r||^2

# Parameters:
# ----------
# alpha          : CoP error squared cost weight            (scalar)
# beta           : CoM position error squared cost weight   (scalar)
# gamma          : CoM velocity error squared cost weight   (scalar)
# N              : preceding horizon                        (scalar)
# P_ps, P_vs     : CoM position recursive dynamics matrices (Nx2 numpy.array,
#                  x^_k+1 = P_ps x^_k + P_pu z_k             NXN numpy.array)
# P_pu , P_vu    : CoM velocity recursive dynamics matrix   (Nx2 numpy.array,
#                  x^dot_k+1 = P_vs x^dot_k + P_vu z_k       NXN numpy.array)
# x_hat_k        : [x^_k, x^dot_k].T current CoM state in x (2, numpy.array)
# y_hat_k        : [y^_k, y^dot_k].T current CoM state in y (2, numpy.array)
# Z_ref_k        : [z_ref_x_k  , z_ref_y_k  ]   CoP reference trajectory
#                   .       ,    .                          (Nx2 numpy.array)
#                   .       ,    .
#                  [z_ref_x_k+N, z_ref_y_k+N]

# Returns:
# -------
# Q     : Hessian  (2Nx2N numpy.array)
# p_k   : Gradient (2N,  numpy.array)

def compute_objective_terms(alpha, beta, gamma, step_duration, no_steps_per_T,
                            N, stride_length, stride_width, P_ps, P_pu, P_vs,
                            P_vu, x_hat_k, y_hat_k, Z_ref_k):

    # pre-allocate memory
    Q    = np.zeros((2*N, 2*N))
    p_k  = np.zeros((2*N))
    Q_prime = np.zeros((N,N))

    Q_prime = alpha*np.eye(N) + (beta * np.dot(P_pu.T, P_pu)) \
              + (gamma * np.dot(P_vu.T, P_vu))
    Q[0:N, 0:N]     = Q_prime  # x-direction
    Q[N:2*N, N:2*N] = Q_prime  # y-direction

    x_r_N = np.zeros((N))
    y_r_N = np.zeros((N))
    x_dotr_N = np.zeros((N))
    y_dotr_N = np.zeros((N))

    x_r_N    = np.tile(stride_length/no_steps_per_T, N)
    y_r_N    = np.tile(stride_width/no_steps_per_T, N)
    x_dotr_N = np.tile(stride_length/step_duration, N)
    y_dotr_N = np.tile(stride_width/step_duration, N)

    p_k[0:N] = gamma * (np.dot(P_vu.T, (np.dot(P_vs, x_hat_k)))- np.dot(P_vu.T, x_dotr_N)) \
               + beta  * (np.dot(P_pu.T, (np.dot(P_ps, x_hat_k)))- np.dot(P_pu.T, x_r_N)) \
               - alpha*Z_ref_k[:,0]

    p_k[N:2*N]  = gamma*(np.dot(P_vu.T, (np.dot(P_vs, y_hat_k)))- np.dot(P_vu.T, y_dotr_N)) \
                  + beta *(np.dot(P_pu.T, (np.dot(P_ps, y_hat_k)))- np.dot(P_pu.T, y_r_N)) \
                  - alpha*Z_ref_k[:,1]
    return Q, p_k

# ------------------------------------------------------------------------------
# unit test: A.K.A red pill or blue pill
# ------------------------------------------------------------------------------
if __name__=='__main__':
    import numpy.random as random
    import motion_model
    print(' visualize your matrices like a Neo ! '.center(60,'*'))

    delta_t = 0.1
    h       = 0.80
    g       = 9.81
    alpha   = 1
    gamma   = 1
    beta    = 0
    N       = 16
    Z_ref_k = random.rand(N,2)-0.5
    x_hat_k = random.rand(2)-0.5
    y_hat_k = random.rand(2)-0.5
    [P_ps, P_vs, P_pu, P_vu] = motion_model.compute_recursive_matrices(delta_t, g, h, N)
    Q, p = compute_objective_terms(alpha, beta, gamma, N, P_ps, P_pu, P_vs, P_vu,
                                x_hat_k, y_hat_k, Z_ref_k)
    p    = np.reshape(p, (p.size,1))

# ------------------------------------------------------------------------------
# visualize your Hessian and gradient like a Neo:
# ------------------------------------------------------------------------------
    with np.nditer(Q, op_flags=['readwrite']) as it:
        for x in it:
            if x[...] != 0:
                x[...] = 1
    plt.figure(1)
    plt.suptitle('Structure of hessian matrix Q')
    plt.imshow(Q, cmap='Greys', extent=[0,Q.shape[1],Q.shape[0],0],
    interpolation = 'nearest')
    #with np.nditer(p, op_flags=['readwrite']) as it:
    #    for x in it:
    #        if x[...] != 0:
    #            x[...] = 1
    plt.figure(2)
    plt.imshow(p, cmap='Greys',  extent=[0,p.shape[1],p.shape[0],0],
    interpolation = 'nearest', aspect=0.25)
    plt.suptitle('Structure of gradient vector P')
    plt.show()

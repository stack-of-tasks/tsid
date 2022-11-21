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

import numpy as np

# Desctiption:
# -----------
# this function assembles A and b encapsulating the ZMP inequality constraints
# in the Quadratic Program of this form (quadprog solver format):
#        minimize
#            (1/2) * u.T * Q * u - p.T * u
#        subject to
#            A.T * u >= b

# Parameters:
# ----------
# N           : preceding horizon (scalar)
# foot_length : length of the foot along the x-axis (scalar)
# foot_width  : length of the foot along the y-axis (scalar)
# P_zs        : (Nx3 numpy.array)
# P_zu        : (NxN numpy.array)
# Z_ref       : [z_ref_x_k  , z_ref_y    ]   CoP reference trajectory
#                   .       ,    .           (Nx2 numpy.array)
#                   .       ,    .
#               [z_ref_x_k+N, z_ref_y_k+N]

# Returns:
# -------
# A  : (4Nx2N numpy.array)
#      matrix defining the constraints under which we want to minimize the
#      quadratic function

# b  : (4N, numpy.array)
#      vector defining the ZMP constraints according to the foot size

# x_hat_k:= [x_k, x_dot_k, x_ddot_k].T current state in x (3, numpy.array)
# y_hat_k:= [y_k, y_dot_k, y_ddot_k].T current state in y (3, numpy.array)

def add_ZMP_constraints(N, foot_length, foot_width, P_zs, P_zu, Z_ref_k, x_hat_k, y_hat_k):
    A = np.zeros((4*N, 2*N))
    b = np.zeros((4*N))

    A[0:N  , 0:N]   = P_zu
    A[N:2*N, 0:N]   = -P_zu

    A[2*N:3*N, N:2*N] = P_zu
    A[3*N:4*N, N:2*N] = -P_zu

    foot_length_N = np.zeros((N))
    foot_width_N  = np.zeros((N))
    foot_length_N = np.tile(foot_length,(N))
    foot_width_N  = np.tile(foot_width,(N))

    b[0:N]     = -np.dot(P_zs, x_hat_k) + Z_ref_k[:,0] - (0.5*foot_length_N)
    b[N:2*N]   = np.dot(P_zs, x_hat_k)  - Z_ref_k[:,0] - (0.5*foot_length_N)
    b[2*N:3*N] = -np.dot(P_zs, y_hat_k) + Z_ref_k[:,1] - (0.5*foot_width_N)
    b[3*N:4*N] =  np.dot(P_zs, y_hat_k) - Z_ref_k[:,1] - (0.5*foot_width_N)

    return A,b

def add_terminal_constraints(N, terminal_index, x_hat_k, y_hat_k, x_terminal, \
                             y_terminal, P_ps, P_vs, P_as, P_pu, P_vu, P_au):

    A = np.zeros((6, 2*N))
    b = np.zeros((6))

    P_ps_current = P_ps[terminal_index,:]
    P_vs_current = P_vs[terminal_index,:]
    P_as_current = P_as[terminal_index,:]

    P_pu_current = P_pu[terminal_index,:]
    P_vu_current = P_vu[terminal_index,:]
    P_au_current = P_au[terminal_index,:]

    A[0, 0:N]    = P_pu_current
    A[1, 0:N]    = P_vu_current
    A[2, 0:N]    = P_au_current
    A[3, N:2*N]  = P_pu_current
    A[4, N:2*N]  = P_vu_current
    A[5, N:2*N]  = P_au_current

    b[0] = x_terminal[0] - np.dot(P_ps_current, x_hat_k)
    b[1] = x_terminal[1] - np.dot(P_vs_current, x_hat_k)
    b[2] = x_terminal[2] - np.dot(P_as_current, x_hat_k)
    b[3] = y_terminal[0] - np.dot(P_ps_current, y_hat_k)
    b[4] = y_terminal[1] - np.dot(P_vs_current, y_hat_k)
    b[5] = y_terminal[2] - np.dot(P_as_current, y_hat_k)

    return A, b

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

# Description:
# -----------
# this function compute the canonical the quadratic objective term Q (hessian)
# and the linear objective term p.T (gradient) inside the cost function of the
# Quadratic Program of this form (quadprog solver format):
#        minimize
#            (1/2) * u.T * Q * u - p.T * u
#        subject to
#            A.T * u >= b

# Parameters:
# ----------
# alpha  : CoM jerk  squared cost weight  (scalar)
# gamma  : CoP error squared cost weight (scalar)
# N      : preceding horizon (scalar)
# P_zs   : (Nx3 numpy.array)
# P_zu   : (Nx3 numpy.array)
# x_hat_k:= [x_k, x_dot_k, x_ddot_k].T current state in x (3, numpy.array)
# y_hat_k:= [y_k, y_dot_k, y_ddot_k].T current state in y (3, numpy.array)
# Z_ref  := [z_ref_x_k  , z_ref_y_k    ]   CoP reference trajectory
#                   .   ,    .           (Nx2 numpy.array)
#                   .   ,    .
#           [z_ref_x_k+N, z_ref_y_k+N]
# Returns:
# -------
# Q     : (2Nx2N numpy.array)
# p_k   : (2N,  numpy.array)

def compute_objective_terms(alpha, gamma, N,  P_zs, P_zu, x_hat_k, y_hat_k, Z_ref_k):
    Q_prime         = np.zeros((N,N))
    Q_prime         = alpha*np.identity(N) + (gamma * np.dot(P_zu.T, P_zu))
    Q               = np.zeros((2*N, 2*N))
    Q[0:N, 0:N]     = Q_prime
    Q[N:2*N, N:2*N] = Q_prime

    p_k             = np.zeros((2*N))
    p_k[0:N]        = gamma * (np.dot(P_zu.T, (np.dot(P_zs, x_hat_k) - Z_ref_k[:,0])))
    p_k[N:2*N]      = gamma * (np.dot(P_zu.T, (np.dot(P_zs, y_hat_k) - Z_ref_k[:,1])))

    return Q, p_k

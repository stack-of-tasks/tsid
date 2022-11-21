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
import math
from numpy.linalg import matrix_power
from scipy.linalg import toeplitz

# Description:
# -----------
# this function returns the discrete dynamics matrices A_d and B_d
# of the linear inverted pendulum x+ = A_d x + B_d x

# Parameters:
# ----------
# delta_t: sampling time
# g      : norm of the gravity acceleration vector
# h      : fixed height of the CoM assuming walking on a flat terrain

# Returns:
# -------
# A_d (2x2 numpy.array)
# B_d (2,  numpy.array)

def discrete_LIP_dynamics(delta_t, g, h):
    w   = math.sqrt(g/h)
    A_d = np.array([[math.cosh(w*delta_t)  , (1/w)*math.sinh(w*delta_t)], \
                    [w*math.sinh(w*delta_t), math.cosh(w*delta_t)]])

    B_d = np.array([1 - math.cosh(w*delta_t), -w*math.sinh(w*delta_t)])

    return A_d, B_d

# Description:
# -----------
# this function computes the integration of the discrete dynamic matrices of the
# linear inverted pendulum. the matrices are constructed once offline since
# all parameters used in the computation are fixed.
#  x^_k+1    = P_ps x^_k    + P_pu z_k
#  x^dot_k+1 = P_vs x^dot_k + P_vu z_k

# Parameters:
# ----------
# delta_t: sampling time
# g      : norm of the gravity acceleration vector
# h      : fixed height of the CoM assuming walking on a flat terrain

# Returns:
# -------
# P_ps, P_vs  : position and velocity partitions of the states recursive
#                dynamics matrices (Nx2 numpy.array, NXN numpy.array)
# P_pu , P_vu : position and velocity partitions of the control inputs recursive
#                dynamics matrix   (Nx2 numpy.array, NXN numpy.array)

def compute_recursive_matrices(delta_t, g, h, N):
    [A_d, B_d] = discrete_LIP_dynamics(delta_t, g, h)

    # pre-allocate memmory
    P_ps       = np.zeros((N,2))
    P_vs       = np.zeros((N,2))
    temp_pu    = np.zeros((N))
    temp_vu    = np.zeros((N))

    for i in range(N):
        A_d_pow     = matrix_power(A_d, i+1)
        P_ps[i,0:2] = A_d_pow[0,:]
        P_vs[i,0:2] = A_d_pow[1,:]
        temp_u      = np.dot(matrix_power(A_d, i), B_d)
        temp_pu[i]  = temp_u[0]
        temp_vu[i]  = temp_u[1]

    P_pu = toeplitz(temp_pu) * np.tri(N,N)
    P_vu = toeplitz(temp_vu) * np.tri(N,N)

    return P_ps, P_vs, P_pu, P_vu

# Description:
# -----------
# this function computes the integration of the recursive discrete dynamics
# of the linear inverted pendulum based on the current initial CoM state.

# Parameters:
# ----------
# P_ps, P_vs  : position and velocity partitions of the states recursive
#                dynamics matrices          (Nx2 numpy.array, NXN numpy.array)
# P_pu , P_vu : position and velocity partitions of the control inputs recursive
#                dynamics matrix            (Nx2 numpy.array, NXN numpy.array)
# N           : preceding horizon           (scalar)
# x_hat_k     : [x^_k, x^dot_k].T current CoM state in x (2, numpy.array)
# y_hat_k     : [y^_k, y^dot_k].T current CoM state in y (2, numpy.array)
# U_k         : [zx_k, ... , zx_k+N, ... ,zy_k, ..., zy_k+N].T
#               current CoP control inputs in x, y directions
#               along the horizon (2N, numpy.array)

# Returns:
# -------
# X: [x_k+1   , x_dot_k+1  ]   recursive CoM dynamics in x direction
#       .     ,   .            (Nx2 numpy.array)
#       .     ,   .
#    [x_k+1+N , x_dot_k+1+N]

# Y: [y_k+1   , y_dot_k+1  ]   recursive CoM dynamics in y direction
#       .     ,   .            (Nx2 numpy.array)
#       .     ,   .
#    [y_k+1+N , y_dot_k+1+N]

def compute_recursive_dynamics(P_ps, P_vs, P_pu, P_vu, N, x_hat_k, y_hat_k, U_k):
    # pre-allocate memory
    X         = np.zeros((N,2))
    Y         = np.zeros((N,2))

    # evaluate your CoM states in the x-direction along the horizon
    X[0:N,0]  = np.dot(P_ps, x_hat_k) + np.dot(P_pu, U_k[0:N])   #x
    X[0:N,1]  = np.dot(P_vs, x_hat_k) + np.dot(P_vu, U_k[0:N])   #x_dot

    # evaluate your CoM states in the y-direction along the horizon
    Y[0:N,0]  = np.dot(P_ps, y_hat_k) + np.dot(P_pu, U_k[N:2*N]) #y
    Y[0:N,1]  = np.dot(P_vs, y_hat_k) + np.dot(P_vu, U_k[N:2*N]) #y_dot

    return X, Y

# ------------------------------------------------------------------------------
# unit test: A.K.A red pill or blue pill
# ------------------------------------------------------------------------------
if __name__=='__main__':
    import numpy.random as random
    print(' Test compute_recursive_matrices '.center(60,'*'))
    h           = 0.80
    g           = 9.81
    delta_t     = 0.1
    N           = 4
    x_0 = random.rand(2) -0.5
    y_0 = random.rand(2) - 0.5
    U   = random.rand(2*N) - 0.5

    [A_d, B_d] = discrete_LIP_dynamics(delta_t, g, h)
    P_ps, P_vs, P_pu, P_vu = compute_recursive_matrices(delta_t, g, h, N)
    X, Y = compute_recursive_dynamics(P_ps, P_vs, P_pu, P_vu, N, x_0, y_0, U)

    X_real,      Y_real         = np.zeros((N,2)), np.zeros((N,2))
    X_real[0,:], Y_real[0,:]    = x_0, y_0

    # recursive integration of dynamics
    for i in range(N-1):
        X_real[i+1,:] = A_d.dot(X_real[i,:]) + B_d.dot(U[i])
        Y_real[i+1,:] = A_d.dot(Y_real[i,:]) + B_d.dot(U[N+i])

    #print 'X:\n', X
    #print 'X_real:\n', X_real

    print('Error on X',     np.max(np.abs(X[:-1,:]-X_real[1:,:])))
    print('Error on Y',     np.max(np.abs(Y[:-1,:]-Y_real[1:,:])))

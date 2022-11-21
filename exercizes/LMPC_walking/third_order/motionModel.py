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
from scipy.linalg import toeplitz

# Description:
# -----------
# this functon computes the recursive integration matrices of the preview system
# this function is computed once offline since all parameters are all fixed
# before the beginning of the optimization

# Parameters:
# ----------
# N: preceding horizon
# T: sampling time
# h: fixed height of the CoM assuming walking on a flat terrain
# g: norm of the gravity acceleration vector

# Returns:
# -------
# P_ps, P_vs, P_as, P_zs (Nx3 numpy.array)
# P_pu, P_vu, P_au, P_zu (N-1 x N-1 numpy.array)

def compute_recursive_matrices(N, T, h, g):
    P_ps = np.zeros((N,3))
    P_vs = np.zeros((N,3))
    P_as = np.zeros((N,3))
    P_zs = np.zeros((N,3))

    temp_pu = np.zeros((N))
    temp_vu = np.zeros((N))
    temp_au = np.zeros((N))
    temp_zu = np.zeros((N))

    for i in range(N):
         P_ps[i, 0:3] = np.array([1.0, (i+1.0)*T, 0.5*((i+1.0)*T)**2.0])
         P_vs[i, 0:3] = np.array([0.0,      1.0 , (i+1.0)*T])
         P_as[i, 0:3] = np.array([0.0,      0.0 , 1.0])
         P_zs[i, 0:3] = np.array([1.0, (i+1.0)*T, 0.5*(((i+1.0)*T)**2.0) - h/g])


         temp_pu[i] = np.array([(1.0 + (3.0*i) + 3.0*((i)**2.0))*(T**3.0)/6.0])
         temp_vu[i] = np.array([(1.0 + (2.0*i)) *(T**2.0)*0.5])
         temp_au[i] = np.array([T])
         temp_zu[i] = np.array([(1.0 + (3.0*i) + 3.0*((i)**2.0))*((T**3.0)/6.0) -T*h/g])

    P_pu = toeplitz(temp_pu) * np.tri(N,N)
    P_vu = toeplitz(temp_vu) * np.tri(N,N)
    P_au = toeplitz(temp_au) * np.tri(N,N)
    P_zu = toeplitz(temp_zu) * np.tri(N,N)

    return P_ps, P_vs, P_as, P_zs, P_pu, P_vu, P_au, P_zu

# Description:
# -----------
# this functon computes the future states and computed CoP along the preceding
# horizon based on the current state and input

# X_k+1   = P_ps * x_hat_k + P_pu * U_x
# Y_k+1   = P_ps * y_hat_k + P_pu * U_y
# Z_x_k+1 = P_zs * x_hat_k + P_zu * U_x
# Z_y_k+1 = P_zs * y_hat_k + P_zu * U_y


# Parameters:
# ----------
# P_ps, P_vs, P_as, P_zs                (Nx3 numpy.array)
# P_pu, P_vu, P_au, P_zu                (N x N numpy.array)
# N: preceding horizon                  (scalar)

# x_hat_k:= [x_k, x_dot_k, x_ddot_k].T   current x state (3, numpy.array)
# y_hat_k:= [y_k, y_dot_k, y_ddot_k].T   current y state (3, numpy.array)

# U      := [x_jerk_k, ... , x_jerk_k+N, ... , y_jerk_k, ..., y_jerk_k+N].T
#           current control inputs (x and y jerks)
#           along the horizon (2N, numpy.array)

# Returns:
# -------
# X:= [x_k+1   , x_dot_k+1   , x_ddot_k+1]  referred above as X_k+1 (Nx3 numpy.array)
#       .      ,   .         , .
#       .      ,   .         , .
#     [x_k+1+N , x_dot_k+1+N , x_ddot_k+1+N]  referred above as Y_k+1 (Nx3 numpy.array)

# Y:= [y_k+1   , y_dot_k+1   , y_ddot_k+1]
#       .      ,   .         , .
#       .      ,   .         , .
#     [y_k+1+N , y_dot_k+1+N , y_ddot_k+1+N]

# Z_x:= [z_x_k+1, ... , z_x_k+1+N].T    referred as Z_x_k+1 above (Nx1 numpy.array)
# Z_y:= [z_y_k+1, ... , z_y_k+1+N].T    referred as Z_y_k+1 above (Nx1 numpy.array)

def compute_recursive_dynamics(P_ps, P_vs, P_as, P_zs, P_pu, P_vu, P_au, P_zu,\
                               N, x_hat_k, y_hat_k, U):
    X         = np.zeros((N,3))
    Y         = np.zeros((N,3))

    # evaluate your CoM states along the horizon
    X[0:N,0]  = np.dot(P_ps, x_hat_k) + np.dot(P_pu, U[0:N])   #x
    X[0:N,1]  = np.dot(P_vs, x_hat_k) + np.dot(P_vu, U[0:N])   #x_dot
    X[0:N,2]  = np.dot(P_as, x_hat_k) + np.dot(P_au, U[0:N])   #x_ddot

    Y[0:N,0]  = np.dot(P_ps, y_hat_k) + np.dot(P_pu, U[N:2*N]) #y
    Y[0:N,1]  = np.dot(P_vs, y_hat_k) + np.dot(P_vu, U[N:2*N]) #y_dot
    Y[0:N,2]  = np.dot(P_as, y_hat_k) + np.dot(P_au, U[N:2*N]) #y_ddot

    # evaluate computed CoP
    Z_x = np.dot(P_zs, x_hat_k) + np.dot(P_zu, U[0:N])
    Z_y = np.dot(P_zs, y_hat_k) + np.dot(P_zu, U[N:2*N])

    return X, Y, Z_x, Z_y
    
    
if __name__=='__main__':
    import numpy.random as random
    print ' Test compute_recursive_matrices '.center(60,'*')
    h           = 0.80
    g           = 9.81
    T           = 0.1                # sampling time interval
    N           = 100   
    x_0 = random.rand(3) -0.5
    y_0 = random.rand(3) - 0.5
    U = random.rand(2*N) - 0.5
    
    [P_ps, P_vs, P_as, P_zs, P_pu, P_vu, P_au, P_zu] = compute_recursive_matrices(N, T, h, g)
    X, Y, Z_x, Z_y = compute_recursive_dynamics(P_ps, P_vs, P_as, P_zs, 
                                                P_pu, P_vu, P_au, P_zu,
                                                N, x_0, y_0, U)
                
    A = np.array([[1.0,   T, 0.5*(T**2.0)],
                  [0.0, 1.0, T           ],
                  [0.0, 0.0, 1.0         ]])
    B = np.array([[(T**3)/6.0, (T**2)/2.0, T]]).T
    C = np.array([[1.0, 0.0, -h/g]])
    
    X_real,      Y_real         = np.zeros((3,N)), np.zeros((3,N))
    Z_x_real,    Z_y_real       = np.zeros((N)), np.zeros((N))
    X_real[:,0], Y_real[:,0]    = x_0, y_0
    for i in range(N-1):
        X_real[:,i+1] = A.dot(X_real[:,i]) + B.dot(U[i]).squeeze()
        Y_real[:,i+1] = A.dot(Y_real[:,i]) + B.dot(U[N+i]).squeeze()
        Z_x_real[i+1] = C.dot(X_real[:,i+1])
        Z_y_real[i+1] = C.dot(Y_real[:,i+1])

#    print 'X:\n', X
#    print 'X_real:\n', X_real.T
    print 'Error on X',     np.max(np.abs(X[:-1,:]-X_real[:,1:].T))
    print 'Error on Y',     np.max(np.abs(Y[:-1,:]-Y_real[:,1:].T))
    print 'Error on ZMP X', np.max(np.abs(Z_x[:-1]-Z_x_real[1:].T))
    print 'Error on ZMP Y', np.max(np.abs(Z_y[:-1]-Z_y_real[1:].T))

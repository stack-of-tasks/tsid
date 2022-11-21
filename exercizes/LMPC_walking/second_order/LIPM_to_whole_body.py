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
from LMPC_walking.second_order.motion_model import discrete_LIP_dynamics

# Generate trajectory using 3rd order polynomial with following constraints:
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
def compute_3rd_order_poly_traj(x0, x1, T, dt):
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


def compute_foot_traj(foot_steps, N, dt, step_time, step_height, first_phase):
    x = np.zeros((3,N+1))
    dx = np.zeros((3,N+1))
    ddx = np.zeros((3,N+1))
    N_step = int(step_time/dt)
    offset = 0
    if first_phase=='swing':
        offset = N_step
        x[0, :N_step] = foot_steps[0,0]
        x[1, :N_step] = foot_steps[0,1]
        
    for s in range(foot_steps.shape[0]):
        i = offset+s*2*N_step
        x[0, i:i+N_step] = foot_steps[s,0]
        x[1, i:i+N_step] = foot_steps[s,1]
        if s<foot_steps.shape[0]-1:
            next_step = foot_steps[s+1,:]
        elif first_phase=='swing':
            break
        else:
            next_step = foot_steps[s,:]
            step_height = 0.0
        x[:2,   i+N_step : i+2*N_step], \
        dx[:2,  i+N_step : i+2*N_step], \
        ddx[:2, i+N_step : i+2*N_step] = \
            compute_3rd_order_poly_traj(foot_steps[s,:], next_step, step_time, dt)
            
        x[2,   i+N_step : i+int(1.5*N_step)], \
        dx[2,  i+N_step : i+int(1.5*N_step)], \
        ddx[2, i+N_step : i+int(1.5*N_step)] = \
            compute_3rd_order_poly_traj(np.array([0.]), np.array([step_height]), 0.5*step_time, dt)
        
        x[2,   i+int(1.5*N_step):i+2*N_step], \
        dx[2,  i+int(1.5*N_step):i+2*N_step], \
        ddx[2, i+int(1.5*N_step):i+2*N_step] = \
            compute_3rd_order_poly_traj(np.array([step_height]), np.array([0.0]), 0.5*step_time, dt)
            
    return x, dx, ddx


def interpolate_lipm_traj(T_step, nb_steps, dt_mpc, dt_ctrl, com_z, g,
                          com_state_x, com_state_y, cop_ref, cop_x, cop_y):
    # INTERPOLATE WITH TIME STEP OF CONTROLLER (TSID)
    N  = nb_steps * int(round(T_step/dt_mpc))  # number of time steps for traj-opt
    N_ctrl = int((N*dt_mpc)/dt_ctrl)   # number of time steps for TSID
    com  = np.empty((3,N_ctrl+1))*np.nan
    dcom = np.zeros((3,N_ctrl+1))
    ddcom = np.zeros((3,N_ctrl+1))
    cop = np.empty((2,N_ctrl+1))*np.nan
    foot_steps = np.empty((2,N_ctrl+1))*np.nan
    contact_phase = (N_ctrl+1)*['right']
    com[2,:] = com_z
    
    N_inner = int(N_ctrl/N)
    for i in range(N):
        com[0,i*N_inner] = com_state_x[i,0]
        com[1,i*N_inner] = com_state_y[i,0]
        dcom[0,i*N_inner] = com_state_x[i,1]
        dcom[1,i*N_inner] = com_state_y[i,1]
        if(i>0): 
            if np.linalg.norm(cop_ref[i,:] - cop_ref[i-1,:]) < 1e-10:
                contact_phase[i*N_inner] = contact_phase[i*N_inner-1]
            else:
                if contact_phase[(i-1)*N_inner]=='right':
                    contact_phase[i*N_inner] = 'left'
                elif contact_phase[(i-1)*N_inner]=='left':
                    contact_phase[i*N_inner] = 'right'
                    
        for j in range(N_inner):
            ii = i*N_inner + j
            (A,B) = discrete_LIP_dynamics((j+1)*dt_ctrl, g, com_z)
            foot_steps[:,ii] = cop_ref[i,:].T
            cop[0,ii] = cop_x[i]
            cop[1,ii] = cop_y[i]
            x_next = A.dot(com_state_x[i,:]) + B.dot(cop[0,ii])
            y_next = A.dot(com_state_y[i,:]) + B.dot(cop[1,ii])
            com[0,ii+1] = x_next[0]
            com[1,ii+1] = y_next[0]
            dcom[0,ii+1] = x_next[1]
            dcom[1,ii+1] = y_next[1]
            ddcom[:2,ii] = g/com_z * (com[:2,ii] - cop[:,ii])
            
            if(j>0): contact_phase[ii] = contact_phase[ii-1]
    return com, dcom, ddcom, cop, contact_phase, foot_steps

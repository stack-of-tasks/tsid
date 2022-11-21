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
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# time vs CoP and CoM in x: 'A.K.A run rabbit run !'
# -------------------------------------------------
def plot_x(time, walking_time, min_admissible_CoP, max_admissible_cop, Z_x, X, Z_ref):
    plt.rc('text', usetex = True)
    plt.rc('font', family ='serif')
    ZMP_x_fig = plt.figure()
    plt.plot(time, Z_x, label = r'\textbf{computed CoP}')
    plt.plot(time, Z_ref[:,0])
    plt.plot(time, X[:,0], 'g', label = r'\textbf{CoM}')
    plt.plot(time, min_admissible_CoP[0:walking_time, 0],'r', linestyle = '--',\
             linewidth = 0.5, label = r'\textbf{min CoP}')
    plt.plot(time, max_admissible_cop[0:walking_time, 0],'m', linestyle = '--',\
             linewidth = 0.5, label = r'\textbf{max CoP}')

    plt.xlabel(r'\textbf{time} (s)')
    plt.ylabel(r'\textbf{x, z} (m)')
    plt.legend()
    #plt.title("time from"  ,str(time[0]), str(time[15]))

# time vs CoM_dot in x: 'A.K.A run rabbit run !'
# -------------------------------------------------

# time VS CoP and CoM in y: 'A.K.A what goes up must go down'
# ----------------------------------------------------------
def plot_y(time, walking_time, min_admissible_CoP, max_admissible_cop, Z_y, Y, Z_ref):
    ZMP_y_fig = plt.figure()
    plt.plot(time, Z_y, label = r'\textbf{computed CoP}')
    plt.plot(time, Z_ref[:,1])
    plt.plot(time, Y[0:walking_time,0], 'g', label = r'\textbf{CoM}')
    plt.plot(time, min_admissible_CoP[0:walking_time, 1],'r', linestyle = '--',\
             linewidth = 0.5, label = r'\textbf{min CoP}')
    plt.plot(time, max_admissible_cop[0:walking_time, 1],'m', linestyle = '--',\
             linewidth = 0.5, label = r'\textbf{max CoP}')

    plt.xlabel(r'\textbf{time} (s)')
    plt.ylabel(r'\textbf{y, z} (m)')
    plt.legend()

# plot CoP, CoM in x Vs Cop, CoM in y:
# ------------------------------------
def plot_xy(time, walking_time, foot_length, foot_width, Z_ref, Z_x, Z_y, X, Y):
    ZMP_CoP_xy_fig = plt.figure()
    plt.plot(Z_x, Z_y, 'r', label = r'\textbf{computed CoP}')
    plt.plot( X[0:walking_time,0], Y[0:walking_time,0], 'lime', label = r'\textbf{CoM}')
    currentAxis    = plt.gca()
    for i in range(walking_time):
        current_foot = patches.Rectangle((Z_ref[i,0]-foot_length/2,  \
                                          Z_ref[i,1]-foot_width/2),  \
                                          foot_length, foot_width,   \
                                          linewidth = 0.8,           \
                                          linestyle = '-.',          \
                                          edgecolor = 'b',           \
                                          facecolor = 'none')
        currentAxis.add_patch(current_foot)
    currentAxis.set_xlim([-0.5,5.0])
    currentAxis.set_ylim([-0.5,0.8])
    plt.xlabel(r'\textbf{x} (m)')
    plt.ylabel(r'\textbf{y} (m)')
    plt.legend()
    plt.show()

# plot every single horizon for debugging:
# ---------------------------------------
def plot_horizons(desired_walking_time, N, desired_Z_ref, horizon_data, foot_length, foot_width):
    for i in range(desired_walking_time):
        time_k  = horizon_data[i]['time_k']
        Z_ref_k = horizon_data[i]['zmp_reference']
        X_k     = horizon_data[i]['X_k']
        Y_k     = horizon_data[i]['Y_k']
        Z_x_k   = horizon_data[i]['Z_x_k']
        Z_y_k   = horizon_data[i]['Z_y_k']

        min_admissible_CoP = Z_ref_k - np.tile([foot_length/2, foot_width/2], (N,1))
        max_admissible_cop = Z_ref_k + np.tile([foot_length/2, foot_width/2], (N,1))

        #plot_x(time_k, N, min_admissible_CoP, max_admissible_cop, \
        #                  Z_x_k, X_k, Z_ref_k)
        plot_y(time_k, N, min_admissible_CoP, max_admissible_cop, \
                          Z_y_k, Y_k, Z_ref_k)
        #plot_xy(time_k, N, foot_length, foot_width, desired_Z_ref, \
        #                   Z_x_k, Z_y_k, X_k, Y_k)

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
# this function implements a desired zig-zag fixed foot step plan located
# in the middle of the robot's foot starting with the right foot

# Parameters:
# ----------
#  foot_step_0 : initial foot step location
#            [foot_step_x0, foot_step_y0].T (2x1 numpy.array)
#  no_steps    : number of desired walking foot steps  (scalar)

# Returns:
# -------
#  Foot_steps = [Foot_steps_x, Foot_steps_y].T   foot steps locations
#                                                (no_steps x 2 numpy.array)

def manual_foot_placement(foot_step_0, fixed_step_x, no_steps):
    Foot_steps   = np.zeros((no_steps, 2))
    for i in range(Foot_steps.shape[0]):
        if i == 0:
            Foot_steps[i,:] = foot_step_0
        else:
            Foot_steps[i,0] = Foot_steps[i-1,0] + fixed_step_x
            Foot_steps[i,1] = -Foot_steps[i-1,1]
    return Foot_steps

# Description:
# -----------
# this function computes a CoP reference trajectory based on a desired
# fixed foot step plan, a desired foot step duration and a sampling time

# Parameters:
# ----------
#  no_steps      : number of desired walking foot steps  (scalar)
#  Foot_steps    := [Foot_steps_x, Foot_steps_y]   foot steps locations
#                                                  (no_steps x 2 numpy.array)
# walking_time   : desired walking time duration   (scalar)
# no_steps_per_T : step_duration/T  (scalar)

# Returns:
# -------
# Z_ref  := [z_ref_x_k             , z_ref_y_k              ]   CoP reference trajectory
#                   .              ,    .                       (walking_timex2 numpy.array)
#                   .              ,    .
#           [z_ref_x_k+walking_time, z_ref_y_k+walking_time]

def create_CoP_trajectory(no_steps, Foot_steps, walking_time, no_steps_per_T):
    Z_ref  = np.zeros((walking_time,2))
    j = 0
    for i in range (Foot_steps.shape[0]):
        Z_ref[j:j+no_steps_per_T, :] = Foot_steps[i,:]
        j = j + no_steps_per_T
    return Z_ref

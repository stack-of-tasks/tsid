# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

N_SIMULATION = 4000             # number of time steps simulated
dt = 0.002                      # controller time step
q0 = np.matrix([[ 0. , -1.0,  0.7,  0. ,  0. ,  0. ]]).T  # initial configuration

# REFERENCE SINUSOIDAL TRAJECTORY
amp                  = np.matrix([0*0.02, 0.1, 0.10]).T           # amplitude
phi                  = np.matrix([0.0, 0.5*np.pi, 0.0]).T         # phase
two_pi_f             = 1.4*2*np.pi*np.matrix([1.0, 0.5, 0.5]).T   # frequency (time 2 PI)
offset               = np.matrix([0.0, 0.0, 0.0]).T

w_ee = 1.0                      # weight of end-effector task
w_posture = 1e-3                # weight of joint posture task
w_torque_bounds = 1.0           # weight of the torque bounds
w_joint_bounds = 1.0

kp_ee = 5.0                   # proportional gain of end-effector constraint
kp_posture = 1.0               # proportional gain of joint posture task

tau_max_scaling = 0.4           # scaling factor of torque bounds
v_max_scaling = 0.4

ee_frame_name = "ee_fixed_joint"        # end-effector frame name
ee_task_mask = np.matrix([1., 1, 1, 0, 0, 0]).T

PRINT_N = 500                   # print every PRINT_N time steps
DISPLAY_N = 20                  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [2.582354784011841, 1.620774507522583, 1.0674564838409424, 0.2770655155181885, 0.5401807427406311, 0.6969326734542847, 0.3817386031150818]
SPHERE_RADIUS = 0.03
REF_SPHERE_RADIUS = 0.03
EE_SPHERE_COLOR  = (1, 0.5, 0, 0.5)
EE_REF_SPHERE_COLOR  = (1, 0, 0, 0.5)

ERROR_MSG = 'You should set the environment variable UR5_MODEL_DIR to something like "$DEVEL_DIR/install/share"\n';
path      = os.environ.get('UR5_MODEL_DIR', ERROR_MSG)
urdf      = path + "/ur_description/urdf/ur5_robot.urdf";
srdf      = path + '/ur_description/srdf/ur5_robot.srdf'
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 18 09:47:07 2019

@author: student
"""

import numpy as np
import os
import pinocchio as pin

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

N_SIMULATION = 12000             # number of time steps simulated
dt = 0.002                      # controller time step
nv = 37

lxp = 0.10                          # foot length in positive x direction
lxn = 0.05                          # foot length in negative x direction
lyp = 0.05                          # foot length in positive y direction
lyn = 0.05                          # foot length in negative y direction
lz = 0.07                            # foot sole height with respect to ankle joint
mu = 0.3                            # friction coefficient
fMin = 5.0                          # minimum normal force
fMax = 1000.0                       # maximum normal force
rf_frame_name = "RF_KFE"        # right foot frame name
lf_frame_name = "LF_KFE"        # left foot frame name
rh_frame_name = "RH_KFE"        # right foot frame name
lh_frame_name = "LH_KFE"        # left foot frame name
contactNormal = np.array([0., 0., 1.])   # direction of the normal to the contact surface

w_com = 1.0                     # weight of center of mass task
w_cop = 0.0                     # weight of center of pressure task
w_am = 0.0                      # weight of angular momentum task
w_foot = 1e-1                   # weight of the foot motion task
w_contact = -1.0                # weight of foot in contact (negative means infinite weight)
w_posture = 1e-4                # weight of joint posture task
w_forceRef = 1e-5               # weight of force regularization task
w_torque_bounds = 1.0           # weight of the torque bounds
w_joint_bounds = 0.0

tau_max_scaling = 1.45           # scaling factor of torque bounds
v_max_scaling = 0.8

kp_contact = 10.0               # proportional gain of contact constraint
kp_foot = 10.0                  # proportional gain of contact constraint
kp_com = 10.0                   # proportional gain of center of mass task
kp_am = 10.0                   # proportional gain of angular momentum task
kp_posture = 1.0               # proportional gain of joint posture task
gain_vector = kp_posture*np.ones(nv-6)
masks_posture = np.ones(nv-6)

viewer = pin.visualize.MeshcatVisualizer
PRINT_N = 500                   # print every PRINT_N time steps
DISPLAY_N = 20                  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [4.0, -0.2, 0.4, 0.5243823528289795, 0.518651008605957, 0.4620114266872406, 0.4925136864185333]

SPHERE_RADIUS = 0.03
REF_SPHERE_RADIUS = 0.03
COM_SPHERE_COLOR  = (1, 0.5, 0, 0.5)
COM_REF_SPHERE_COLOR  = (1, 0, 0, 0.5)
RF_SPHERE_COLOR  = (0, 1, 0, 0.5)
RF_REF_SPHERE_COLOR  = (0, 1, 0.5, 0.5)
LF_SPHERE_COLOR  = (0, 0, 1, 0.5)
LF_REF_SPHERE_COLOR  = (0.5, 0, 1, 0.5)

# filename = str(os.path.dirname(os.path.abspath(__file__)))
path = '/home/kian/catkin_ws/src/example-robot-data/robots/anymal_b_simple_description'
urdf = path + '/robots/anymal-kinova.urdf'
srdf = path + '/srdf/anymal-kinova.srdf'
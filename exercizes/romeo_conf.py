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

lxp = 0.10                          # foot length in positive x direction
lxn = 0.05                          # foot length in negative x direction
lyp = 0.05                          # foot length in positive y direction
lyn = 0.05                          # foot length in negative y direction
lz = 0.105                          # foot sole height with respect to ankle joint
mu = 0.3                            # friction coefficient
fMin = 5.0                          # minimum normal force
fMax = 1000.0                       # maximum normal force
rf_frame_name = "RAnkleRoll"        # right foot frame name
lf_frame_name = "LAnkleRoll"        # left foot frame name
contactNormal = np.matrix([0., 0., 1.]).T   # direction of the normal to the contact surface

w_com = 1.0                     # weight of center of mass task
w_foot = 1e-1                   # weight of the foot motion task
w_posture = 1e-3                # weight of joint posture task
w_forceRef = 1e-5               # weight of force regularization task

kp_contact = 10.0               # proportional gain of contact constraint
kp_foot = 10.0                  # proportional gain of contact constraint
kp_com = 10.0                   # proportional gain of center of mass task
kp_posture = 10.0               # proportional gain of joint posture task

PRINT_N = 500                   # print every PRINT_N time steps
DISPLAY_N = 25                  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [3.0, -0.2, 0.4, 0.5243823528289795, 0.518651008605957, 0.4620114266872406, 0.4925136864185333]

SPHERE_RADIUS = 0.03
REF_SPHERE_RADIUS = 0.03
COM_SPHERE_COLOR  = (1, 0.5, 0, 1)
COM_REF_SPHERE_COLOR  = (1, 0, 0, 1)
RF_SPHERE_COLOR  = (0, 1, 0, 1)
RF_REF_SPHERE_COLOR  = (0, 1, 0.5, 1)
LF_SPHERE_COLOR  = (0, 0, 1, 1)
LF_REF_SPHERE_COLOR  = (0.5, 0, 1, 1)

filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + '/../models/romeo'
urdf = path + '/urdf/romeo.urdf'
srdf = path + '/srdf/romeo_collision.srdf'
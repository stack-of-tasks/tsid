import os
import pinocchio as pin
import numpy as np
from example_robot_data.robots_loader import getModelPath

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

N_SIMULATION = 4000  # number of time steps simulated
dt = 0.002           # controller time step

lxp = 0.10                                  # foot length in positive x direction
lxn = 0.10                                  # foot length in negative x direction
lyp = 0.065                                  # foot length in positive y direction
lyn = 0.065                                  # foot length in negative y direction
lz = 0.                                     # foot sole height with respect to ankle joint
mu = 0.3                                    # friction coefficient
fMin = 5.0                                  # minimum normal force
fMax = 1000.0                               # maximum normal force
rf_frame_name = "leg_right_sole_fix_joint"  # right foot frame name
lf_frame_name = "leg_left_sole_fix_joint"   # left foot frame name
contactNormal = np.array([0., 0., 1.])      # direction of the normal to the contact surface

w_com = 1.0             # weight of center of mass task
w_am = 1e-3             # weight of angular momentum task
w_foot = 1e-1           # weight of the foot motion task
w_contact = -1.0        # weight of foot in contact (negative means infinite weight)
w_posture = 1e-1        # weight of joint posture task
w_forceRef = 1e-5       # weight of force regularization task
w_cop = 0.0
w_torque_bounds = 1.0   # weight of the torque bounds
w_joint_bounds = 0.0

gain_vector = np.array(  # gain vector for postural task
    [
        10.,
        5.,
        5.,
        1.,
        1.,
        10.,  # lleg  #low gain on axis along y and knee
        10.,
        5.,
        5.,
        1.,
        1.,
        10.,  #rleg
        5000.,
        5000.,  #chest
        500.,
        1000.,
        10.,
        10.,
        10.,
        10.,
        100.,
        50.,  #larm
        50.,
        100.,
        10.,
        10.,
        10.,
        10.,
        100.,
        50.,  #rarm
        100.,
        100.
    ]  #head
)
masks_posture = np.ones(32)

tau_max_scaling = 1.45  # scaling factor of torque bounds
v_max_scaling = 0.8

kp_contact = 10.0       # proportional gain of contact constraint
kp_foot = 10.0          # proportional gain of contact constraint
kp_com = 10.0           # proportional gain of center of mass task
kp_am = 10.0            # proportional gain of angular momentum task
kp_posture = 1.0        # proportional gain of joint posture task

viewer = pin.visualize.MeshcatVisualizer
PRINT_N = 500           # print every PRINT_N time steps
DISPLAY_N = 20          # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [4.0, -0.2, 0.4, 0.5243823528289795, 0.518651008605957, 0.4620114266872406, 0.4925136864185333]

SPHERE_RADIUS = 0.03
REF_SPHERE_RADIUS = 0.03
COM_SPHERE_COLOR = (1, 0.5, 0, 0.5)
COM_REF_SPHERE_COLOR = (1, 0, 0, 0.5)
RF_SPHERE_COLOR = (0, 1, 0, 0.5)
RF_REF_SPHERE_COLOR = (0, 1, 0.5, 0.5)
LF_SPHERE_COLOR = (0, 0, 1, 0.5)
LF_REF_SPHERE_COLOR = (0.5, 0, 1, 0.5)

urdf = "/talos_data/robots/talos_reduced.urdf"
path = getModelPath(urdf)
urdf = path + urdf
srdf = path + '/talos_data/srdf/talos.srdf'
path = os.path.join(path, '../..')

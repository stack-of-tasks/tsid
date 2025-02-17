"""Created on Thu Apr 18 09:47:07 2019

@author: student
"""

from pathlib import Path

import numpy as np
import pinocchio as pin
from example_robot_data.robots_loader import getModelPath

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

DATA_FILE_LIPM = "talos_walking_traj_lipm.npz"
DATA_FILE_TSID = "talos_walking_traj_tsid.npz"

# robot parameters
# ----------------------------------------------
filename = str(Path(__file__).resolve().parent)
urdf = "talos_data/robots/talos_reduced.urdf"
modelPath = Path(getModelPath(urdf))
urdf = modelPath / urdf
srdf = modelPath / "talos_data/srdf/talos.srdf"
path = modelPath / "../.."

nv = 38
foot_scaling = 1.0
lxp = foot_scaling * 0.10  # foot length in positive x direction
lxn = foot_scaling * 0.05  # foot length in negative x direction
lyp = foot_scaling * 0.05  # foot length in positive y direction
lyn = foot_scaling * 0.05  # foot length in negative y direction
lz = 0.0  # foot sole height with respect to ankle joint
mu = 0.3  # friction coefficient
fMin = 0.0  # minimum normal force
fMax = 1e6  # maximum normal force
rf_frame_name = "leg_right_sole_fix_joint"  # right foot frame name
lf_frame_name = "leg_left_sole_fix_joint"  # left foot frame name
contactNormal = np.matrix(
    [0.0, 0.0, 1.0]
).T  # direction of the normal to the contact surface

# configuration for LIPM trajectory optimization
# ----------------------------------------------
alpha = 10 ** (2)  # CoP error squared cost weight
beta = 0  # CoM position error squared cost weight
gamma = 10 ** (-1)  # CoM velocity error squared cost weight
h = 0.88  # fixed CoM height
g = 9.81  # norm of the gravity vector
foot_step_0 = np.array([0.0, -0.085])  # initial foot step position in x-y
dt_mpc = 0.1  # sampling time interval
T_step = 1.2  # time needed for every step
step_length = 0.2  # fixed step length
step_height = 0.05  # fixed step height
nb_steps = 4  # number of desired walking steps

# configuration for TSID
# ----------------------------------------------
dt = 0.002  # controller time step
T_pre = 1.5  # simulation time before starting to walk
T_post = 1.5  # simulation time after walking

w_com = 1.0  # weight of center of mass task
w_foot = 1e0  # weight of the foot motion task
w_contact = 1e2  # weight of the foot in contact
w_posture = 1e-4  # weight of joint posture task
w_forceRef = 1e-5  # weight of force regularization task
w_torque_bounds = 0.0  # weight of the torque bounds
w_joint_bounds = 0.0

tau_max_scaling = 1.45  # scaling factor of torque bounds
v_max_scaling = 0.8

kp_contact = 10.0  # proportional gain of contact constraint
kp_foot = 10.0  # proportional gain of contact constraint
kp_com = 10.0  # proportional gain of center of mass task
kp_posture = 1.0  # proportional gain of joint posture task
# gain_vector = kp_posture*np.ones(nv-6)
gain_vector = np.array(  # gain vector for postural task
    [
        10.0,
        5.0,
        5.0,
        1.0,
        1.0,
        10.0,  # lleg  #low gain on axis along y and knee
        10.0,
        5.0,
        5.0,
        1.0,
        1.0,
        10.0,  # rleg
        5000.0,
        5000.0,  # chest
        500.0,
        1000.0,
        10.0,
        10.0,
        10.0,
        10.0,
        100.0,
        50.0,  # larm
        50.0,
        100.0,
        10.0,
        10.0,
        10.0,
        10.0,
        100.0,
        50.0,  # rarm
        100.0,
        100.0,
    ]  # head
)
masks_posture = np.ones(nv - 6)

# configuration for viewer
# ----------------------------------------------
viewer = pin.visualize.GepettoVisualizer
PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 20  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [
    3.578777551651001,
    1.2937744855880737,
    0.8885031342506409,
    0.4116811454296112,
    0.5468055009841919,
    0.6109083890914917,
    0.3978860676288605,
]

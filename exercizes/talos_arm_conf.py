import numpy as np
from example_robot_data.robots_loader import getModelPath

np.set_printoptions(precision=3, linewidth=200, suppress=True)
LINE_WIDTH = 60

N_SIMULATION = 4000  # number of time steps simulated
dt = 0.002  # controller time step
q0 = np.array([1.73046e-01, -2e-04, -5.25366e-01, 0, 0, 1, 0])

# REFERENCE SINUSOIDAL TRAJECTORY
amp = np.array([0 * 0.02, 0.1, 0.10])  # amplitude
phi = np.array([0.0, 0.5 * np.pi, 0.0])  # phase
two_pi_f = 1.4 * 2 * np.pi * np.array([1.0, 0.5, 0.5])  # frequency (time 2 PI)
offset = np.array([0.0, 0.0, 0.0])

w_ee = 1.0  # weight of end-effector task
w_posture = 1e-3  # weight of joint posture task
w_torque_bounds = 1.0  # weight of the torque bounds
w_joint_bounds = 1.0

kp_ee = 5.0  # proportional gain of end-effector constraint
kp_posture = 1.0  # proportional gain of joint posture task

tau_max_scaling = 0.4  # scaling factor of torque bounds
v_max_scaling = 0.4

ee_frame_name = "ee_fixed_joint"  # end-effector frame name
ee_task_mask = np.array([1., 1, 1, 0, 0, 0])

PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 20  # update robot configuration in viwewer every DISPLAY_N time steps
CAMERA_TRANSFORM = [2.582354784011841, 1.620774507522583, 1.0674564838409424, 0.2770655155181885, 0.5401807427406311,
                    0.6969326734542847, 0.3817386031150818]
SPHERE_RADIUS = 0.03
REF_SPHERE_RADIUS = 0.03
EE_SPHERE_COLOR = (1, 0.5, 0, 0.5)
EE_REF_SPHERE_COLOR = (1, 0, 0, 0.5)

urdf = "/talos_data/robots/talos_left_arm.urdf"
path = getModelPath(urdf)
urdf = path + urdf

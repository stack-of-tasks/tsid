## This file contains a TSID based Inverse Dynamics Controller
## Author : Avadesh Meduri
## Date : 16/03/2021

from tsid_quadruped import TSID_controller
import numpy as np
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper

urdf = '/home/kian/catkin_ws/src/example-robot-data/robots/anymal_b_simple_description/robots/anymal.urdf'
model_path = '/home/kian/catkin_ws/src/example-robot-data/robots/anymal_b_simple_description/'
mesh_dir = '/home/kian/catkin_ws/src/example-robot-data/robots/anymal_b_simple_description/meshes/'

eff_array = ["root_joint","LF_HAA","LF_HFE","LF_KFE","RF_HAA","RF_HFE","RF_KFE","LH_HAA","LH_HFE","LH_KFE","RH_HAA","RH_HFE","RH_KFE"]

q0 = np.array([0, 0, 0.4792, 0, 0, 0, 1, -0.1, 0.7, -1, 0.1, 0.7, -1, -0.1, -0.7, 1, 0.1, -0.7, 1])
v0 = np.zeros(len(q0)-1)

robot = RobotWrapper.BuildFromURDF(urdf, mesh_dir, pin.JointModelFreeFlyer())

controller = TSID_controller(robot, urdf, model_path, eff_array, q0, v0)

dt = 0.001
t = 0 + dt
q_des = q0 + np.array([0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])*0.1
v_des = np.array([0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
a_des = np.array([0, 0, 0, 0, 0, 0, 0, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])

fff = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
des_cnt_array = np.array([0, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1])

controller.compute_id_torques(t, q0, v0, q_des, v_des, a_des, fff, des_cnt_array)

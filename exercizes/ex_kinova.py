import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import plot_utils as plut
import time
import pinocchio as pin
import tsid
import subprocess
import os
from math import sin, cos

import kinova_conf as conf

print("".center(conf.LINE_WIDTH, '#'))
print(" Joint Space Inverse Dynamics - Manipulator ".center(conf.LINE_WIDTH, '#'))
print("".center(conf.LINE_WIDTH, '#'), '\n')

PLOT_JOINT_POS = 1
PLOT_JOINT_VEL = 1
PLOT_JOINT_ACC = 1
PLOT_TORQUES = 0
USE_VIEWER = 1

robot = tsid.RobotWrapper(conf.urdf, [conf.path], False)
model = robot.model()

formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
q0 = conf.q0
v0 = np.zeros(robot.nv)
formulation.computeProblemData(0.0, q0, v0)

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(conf.kp_posture * np.ones(robot.nv))
postureTask.setKd(2.0 * np.sqrt(conf.kp_posture) * np.ones(robot.nv))
formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)

trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q0)
postureTask.setReference(trajPosture.computeNext())

v_max = conf.v_max_scaling * model.velocityLimit
v_min = -v_max
# jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot, conf.dt)
# jointBoundsTask.setVelocityBounds(v_min, v_max)
# formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

if (USE_VIEWER):
    robot_display = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path, ])
    # l = subprocess.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
    # if int(l[1]) == 0:
    #     os.system('gepetto-gui &')
    # time.sleep(1)
    # gepetto.corbaserver.Client()
    # robot_display.initViewer(loadModel=True)
    # robot_display.displayCollisions(False)
    # robot_display.displayVisuals(True)
    # robot_display.display(q0)
#    robot_display.viewer.gui.setCameraTransform(0, conf.CAMERA_TRANSFORM)
    viewer = pin.visualize.MeshcatVisualizer
    viz = viewer(robot_display.model, robot_display.collision_model,
                 robot_display.visual_model)
    viz.initViewer(loadModel=True)
    viz.display(q0)

N = conf.N_SIMULATION
tau = np.empty((robot.na, N)) * nan
q = np.empty((robot.nq, N + 1)) * nan
v = np.empty((robot.nv, N + 1)) * nan
dv = np.empty((robot.nv, N + 1)) * nan
q_ref = np.empty((robot.nq, N)) * nan
v_ref = np.empty((robot.nv, N)) * nan
dv_ref = np.empty((robot.nv, N)) * nan
#dv_des = np.empty((robot.nv, N)) * nan
samplePosture = trajPosture.computeNext()

amp = np.array([0.05, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # amplitude
amp2 = np.array([0.05, 0.05, 0.0, 0.0, 0.0, 0.0])  # amplitude
phi = np.array([0.0, 0*np.pi, 0.0, 0.0, 0.0, 0.0, 0, 0, 0])  # phase
phi6 = np.array([0.0, 0*np.pi, 0.0, 0.0, 0.0, 0.0])  # phase
two_pi_f_9 = 2 * np.pi * np.array([0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0, 0, 0])
two_pi_f = 2 * np.pi * np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # frequency (time 2 PI)
two_pi_f_amp = np.multiply(two_pi_f, amp2)
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

t = 0.0
dt = conf.dt
q[:, 0], v[:, 0] = q0, v0

N2 = N*N

for i in range(0, N):
    time_start = time.time()

    # # set reference trajectory
    # q_ref[:, i] = q0 + amp * np.sin(two_pi_f_9 * t + phi)
    # v_ref[:, i] = two_pi_f_amp * np.cos(two_pi_f * t + phi6)
    # dv_ref[:, i] = -two_pi_f_squared_amp * np.sin(two_pi_f * t + phi6)
    q_ref[:, i] = [cos(i*i/N2), sin(i*i/N2), i*i/N2, i*i/N2, cos(i*i/N2), sin(i*i/N2), i*i/N2, cos(i*i/N2), sin(i*i/N2)]
    v_ref[:, i] = [2*i/N2, 2*i/N2, 2*i/N2, 2*i/N2, 2*i/N2, 2*i/N2]
    dv_ref[:, i] = [4*i*i/N2, 4*i*i/N2, 4*i*i/N2, 4*i*i/N2, 4*i*i/N2, 4*i*i/N2]
    samplePosture.pos(q_ref[:, i])
    samplePosture.vel(v_ref[:, i])
    samplePosture.acc(dv_ref[:, i])
    postureTask.setReference(samplePosture)

    HQPData = formulation.computeProblemData(t, q[:, i], v[:, i])
    sol = solver.solve(HQPData)
    if (sol.status != 0):
        print("Time %.3f QP problem could not be solved! Error code:" % t, sol.status)
        break

    tau[:, i] = formulation.getActuatorForces(sol)
    dv[:, i] = formulation.getAccelerations(sol)
    #dv_des[:, i] = postureTask.getDesiredAcceleration

    if i % conf.PRINT_N == 0:
        print("Time %.3f" % (t))
        print("\ttracking err %s: %.3f" % (postureTask.name.ljust(20, '.'), norm(postureTask.position_error, 2)))

    # numerical integration
    v_mean = v[:, i] + 0.5 * dt * dv[:, i]
    v[:, i + 1] = v[:, i] + dt * dv[:, i]
    q[:, i + 1] = pin.integrate(model, q[:, i], dt * v_mean)
    t += conf.dt

    if i % conf.DISPLAY_N == 0:
        # robot_display.display(q[:, i])
        viz.display(q_ref[:, i])

    time_spent = time.time() - time_start
    if (time_spent < conf.dt): time.sleep(conf.dt - time_spent)

# PLOT STUFF
time = np.arange(0.0, N * conf.dt, conf.dt)

if (PLOT_JOINT_POS):
    (f, ax) = plut.create_empty_figure(int(robot.nv / 2), 2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, q[i, :-1], label='q ' + str(i))
        ax[i].plot(time, q_ref[i, :], '--', label='q ref ' + str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Q [rad]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if (PLOT_JOINT_VEL):
    (f, ax) = plut.create_empty_figure(int(robot.nv / 2), 2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, v[i, :-1], label='dq ' + str(i))
        ax[i].plot(time, v_ref[i, :], '--', label='dq ref ' + str(i))
        ax[i].plot([time[0], time[-1]], 2 * [v_min[i]], ':')
        ax[i].plot([time[0], time[-1]], 2 * [v_max[i]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('dq [rad/s]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if (PLOT_JOINT_ACC):
    (f, ax) = plut.create_empty_figure(int(robot.nv / 2), 2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, dv[i, :-1], label='ddq ' + str(i))
        ax[i].plot(time, dv_ref[i, :], '--', label='ddq ref ' + str(i))
        #ax[i].plot(time, dv_des[i, :], ':', label='ddq des ' + str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('ddq [rad/s^2]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if (PLOT_TORQUES):
    (f, ax) = plut.create_empty_figure(int(robot.nv / 2), 2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, tau[i, :], label='Torque ' + str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Torque [Nm]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

plt.show()


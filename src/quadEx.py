import pinocchio as pin
import tsid
import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import os
# import gepetto.corbaserver
import time
import subprocess
import sys

sys.path += [os.getcwd() + '/../exercizes']
import plot_utils as plut
import matplotlib.pyplot as plt

np.set_printoptions(precision=3, linewidth=200, suppress=True)

LINE_WIDTH = 60
print("".center(LINE_WIDTH, '#'))
print(" Test TSID with Quadruped Robot ".center(LINE_WIDTH, '#'))
print("".center(LINE_WIDTH, '#'), '\n')

mu = 0.3  # friction coefficient
fMin = 1.0  # minimum normal force
fMax = 100.0  # maximum normal force
contact_frames = ['BL_contact', 'BR_contact', 'FL_contact', 'FR_contact']
contactNormal = np.array([0., 0., 1.])  # direction of the normal to the contact surface

w_com = 1.0  # weight of center of mass task
w_posture = 1e-3  # weight of joint posture task
w_forceRef = 1e-5  # weight of force regularization task

kp_contact = 10.0  # proportional gain of contact constraint
kp_com = 10.0  # proportional gain of center of mass task
kp_posture = 10.0  # proportional gain of joint posture task

dt = 0.001  # controller time step
PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 25  # update robot configuration in viwewer every DISPLAY_N time steps
N_SIMULATION = 6000  # number of time steps simulated

filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + '/../models'
urdf = path + '/quadruped/urdf/quadruped.urdf'
vector = pin.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, pin.JointModelFreeFlyer(), False)

# for gepetto viewer
robot_display = pin.RobotWrapper.BuildFromURDF(urdf, [path, ], pin.JointModelFreeFlyer())
# l = subprocess.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
# if int(l[1]) == 0:
#     os.system('gepetto-gui &')
# time.sleep(1)
# cl = gepetto.corbaserver.Client()
# gui = cl.gui
# robot_display.initViewer(loadModel=True)
viewer = pin.visualize.MeshcatVisualizer
viz = viewer(robot_display.model, robot_display.collision_model,
             robot_display.visual_model)
viz.initViewer(loadModel=True)


# model = robot.model()
# pin.loadReferenceConfigurations(model, srdf, False)
# q = model.referenceConfigurations["half_sitting"]
# q = pin.getNeutralConfigurationFromSrdf(robot.model(), srdf, False)
# q = robot_display.model.neutralConfiguration #np.zeros(robot.nq)
q = np.zeros(robot.nq)
q[6] = 1.0
q[2] += 0.5
for i in range(4):
    q[7 + 2 * i] = -0.8
    q[8 + 2 * i] = 1.6
v = np.zeros(robot.nv)

viz.display(q)

robot_display.displayCollisions(False)
robot_display.displayVisuals(True)
robot_display.display(q)

assert [robot.model().existFrame(name) for name in contact_frames]

t = 0.0  # time
invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)
data = invdyn.data()

# Place the robot onto the ground.
id_contact = robot_display.model.getFrameId(contact_frames[0])
q[2] -= robot.framePosition(data, id_contact).translation[2]
robot.computeAllTerms(data, q, v)

contacts = 4 * [None]
for i, name in enumerate(contact_frames):
    contacts[i] = tsid.ContactPoint(name, robot, name, contactNormal, mu, fMin, fMax)
    contacts[i].setKp(kp_contact * np.ones(3))
    contacts[i].setKd(2.0 * np.sqrt(kp_contact) * np.ones(3))
    H_rf_ref = robot.framePosition(data, robot.model().getFrameId(name))
    contacts[i].setReference(H_rf_ref)
    contacts[i].useLocalFrame(False)
    invdyn.addRigidContact(contacts[i], w_forceRef, 1.0, 1)

comTask = tsid.TaskComEquality("task-com", robot)
comTask.setKp(kp_com * np.ones(3))
comTask.setKd(2.0 * np.sqrt(kp_com) * np.ones(3))
invdyn.addMotionTask(comTask, w_com, 1, 0.0)

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture * np.ones(robot.nv - 6))
postureTask.setKd(2.0 * np.sqrt(kp_posture) * np.ones(robot.nv - 6))
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)

com_ref = robot.com(data)
trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
sampleCom = trajCom.computeNext()

q_ref = q[7:]
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)

print("Create QP solver with ", invdyn.nVar, " variables, ", invdyn.nEq,
      " equality and ", invdyn.nIn, " inequality constraints")
solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

com_pos = np.empty((3, N_SIMULATION)) * nan
com_vel = np.empty((3, N_SIMULATION)) * nan
com_acc = np.empty((3, N_SIMULATION)) * nan

com_pos_ref = np.empty((3, N_SIMULATION)) * nan
com_vel_ref = np.empty((3, N_SIMULATION)) * nan
com_acc_ref = np.empty((3, N_SIMULATION)) * nan
com_acc_des = np.empty((3, N_SIMULATION)) * nan  # acc_des = acc_ref - Kp*pos_err - Kd*vel_err

offset = robot.com(data) + np.array([0.0, 0.0, 0.0])
amp = np.array([0.0, 0.03, 0.0])
two_pi_f = 2 * np.pi * np.array([0.0, 2.0, 0.7])
two_pi_f_amp = np.multiply(two_pi_f, amp)
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

for i in range(0, N_SIMULATION):
    time_start = time.time()

    sampleCom.pos(offset + np.multiply(amp, np.sin(two_pi_f * t)))
    sampleCom.vel(np.multiply(two_pi_f_amp, np.cos(two_pi_f * t)))
    sampleCom.acc(np.multiply(two_pi_f_squared_amp, -np.sin(two_pi_f * t)))

    comTask.setReference(sampleCom)
    samplePosture = trajPosture.computeNext()
    postureTask.setReference(samplePosture)

    HQPData = invdyn.computeProblemData(t, q, v)
    if i == 0: HQPData.print_all()

    sol = solver.solve(HQPData)
    if (sol.status != 0):
        print("[%d] QP problem could not be solved! Error code:" % (i), sol.status)
        break

    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)

    com_pos[:, i] = robot.com(invdyn.data())
    com_vel[:, i] = robot.com_vel(invdyn.data())
    com_acc[:, i] = comTask.getAcceleration(dv)
    com_pos_ref[:, i] = sampleCom.pos()
    com_vel_ref[:, i] = sampleCom.vel()
    com_acc_ref[:, i] = sampleCom.acc()
    com_acc_des[:, i] = comTask.getDesiredAcceleration

    if i % PRINT_N == 0:
        print("Time %.3f" % (t))
        print("\tNormal forces: ", end=' ')
        for contact in contacts:
            if invdyn.checkContact(contact.name, sol):
                f = invdyn.getContactForce(contact.name, sol)
                print("%4.1f" % (contact.getNormalForce(f)), end=' ')

        print("\n\ttracking err %s: %.3f" % (comTask.name.ljust(20, '.'), norm(comTask.position_error, 2)))
        print("\t||v||: %.3f\t ||dv||: %.3f" % (norm(v, 2), norm(dv)))

    v_mean = v + 0.5 * dt * dv
    v += dt * dv
    q = pin.integrate(robot.model(), q, dt * v_mean)
    t += dt

    if i % DISPLAY_N == 0: robot_display.display(q)

    time_spent = time.time() - time_start
    if (time_spent < dt): time.sleep(dt - time_spent)

# PLOT STUFF
time = np.arange(0.0, N_SIMULATION * dt, dt)

(f, ax) = plut.create_empty_figure(3, 1)
for i in range(3):
    ax[i].plot(time, com_pos[i, :], label='CoM ' + str(i))
    ax[i].plot(time, com_pos_ref[i, :], 'r:', label='CoM Ref ' + str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM [m]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

(f, ax) = plut.create_empty_figure(3, 1)
for i in range(3):
    ax[i].plot(time, com_vel[i, :], label='CoM Vel ' + str(i))
    ax[i].plot(time, com_vel_ref[i, :], 'r:', label='CoM Vel Ref ' + str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM Vel [m/s]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

(f, ax) = plut.create_empty_figure(3, 1)
for i in range(3):
    ax[i].plot(time, com_acc[i, :], label='CoM Acc ' + str(i))
    ax[i].plot(time, com_acc_ref[i, :], 'r:', label='CoM Acc Ref ' + str(i))
    ax[i].plot(time, com_acc_des[i, :], 'g--', label='CoM Acc Des ' + str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM Acc [m/s^2]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()
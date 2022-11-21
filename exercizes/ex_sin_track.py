# Python import needed by the exercise
import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import norm
import os
import time as tmp

# import the library TSID for the Whole-Body Controller
import tsid
# import the pinocchio library for the mathematical methods (Lie algebra) and multi-body dynamics computations.
import pinocchio as pin
# import example_example_robot_data to get the current path for the robot models
from example_robot_data.path import EXAMPLE_ROBOT_DATA_MODEL_DIR

import sys
sys.path.append('..')

# import graphical tools
import plot_utils as plut

# Definition of the tasks gains, weights and the foot geometry (for contact task)

lxp = 0.1      # foot length in positive x direction
lxn = 0.11     # foot length in negative x direction
lyp = 0.069    # foot length in positive y direction
lyn = 0.069    # foot length in negative y direction
lz = 0.107     # foot sole height with respect to ankle joint
mu = 0.3       # friction coefficient
fMin = 1.0     # minimum normal force
fMax = 1000.0  # maximum normal force

rf_frame_name = "RF_KFE"        # right foot joint name
lf_frame_name = "LF_KFE"        # left foot joint name
contactNormal = np.array([0., 0., 1.])  # direction of the normal to the contact surface

w_com = 1      # weight of center of mass task 1
w_posture = 0.1   # weight of joint posture task 0.1
w_forceRef = 1e-3  # weight of force regularization task 1e-3

kp_contact = 30.0  # proportional gain of contact constraint
kp_com = 20.0      # proportional gain of center of mass task

kp_posture = np.array(                       # proportional gain of joint posture task
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]                              # head
)

dt = 0.001            # controller time step
PRINT_N = 500         # print every PRINT_N time steps
DISPLAY_N = 20        # update robot configuration in viwewer every DISPLAY_N time steps
N_SIMULATION = 10000  # number of time steps simulated

# Set the path where the urdf file of the robot is registered
path = EXAMPLE_ROBOT_DATA_MODEL_DIR
urdf = path + '/anymal_b_simple_description/robots/anymal.urdf'
# Create the robot wrapper from the urdf, it will give the model of the robot and its data
robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
srdf = path + '/anymal_b_simple_description/srdf/anymal.srdf'


# Creation of the robot wrapper for gepetto viewer (graphical interface)
robot_display = pin.RobotWrapper.BuildFromURDF(urdf, [os.path.join(path, '../..')], pin.JointModelFreeFlyer())
#Viewer = pin.visualize.GepettoVisualizer
Viewer = pin.visualize.MeshcatVisualizer
viz = Viewer(robot_display.model, robot_display.collision_model, robot_display.visual_model)
viz.initViewer(loadModel=True)

# Take the model of the robot and load its reference configurations
model = robot.model()
pin.loadReferenceConfigurations(model, srdf, False)
# Set the current configuration q to the robot configuration half_sitting
q = model.referenceConfigurations['standing']
# Set the current velocity to zero
v = np.zeros(robot.nv)

# Display the robot in Gepetto Viewer in the configuration q = halfSitting
viz.display(q)

# Check that the frames of the feet exist.
assert model.existFrame(rf_frame_name)
assert model.existFrame(lf_frame_name)

t = 0.0  # time

# InverseDynamicsFormulationAccForce is the class in charge of
# creating the inverse dynamics HQP problem using
# the robot accelerations (base + joints) and the contact forces as decision variables
invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)

# Get the data -> initial data
data = invdyn.data()

# COM Task
comTask = tsid.TaskComEquality("task-com", robot)
comTask.setKp(kp_com * np.ones(3)) # Proportional gain defined before = 20
comTask.setKd(2.0 * np.sqrt(kp_com) * np.ones(3)) # Derivative gain = 2 * sqrt(20)
# Add the task to the HQP with weight = 1.0, priority level = 0 (as constraint) and a transition duration = 0.0
invdyn.addMotionTask(comTask, w_com, 0, 0.0)


# Add a Mask to the task which will select the vector dimensions on which the task will act.
# In this case the waist configuration is a vector 6d (position and orientation -> SE3)
# Here we set a mask = [0 0 0 1 1 1] so the task on the waist will act on the orientation of the robot
# mask = np.ones(6)
# mask[:3] = 0.
# waistTask.setMask(mask)
# Add the task to the HQP with weight = 1.0, priority level = 1 (in the cost function) and a transition duration = 0.0
# invdyn.addMotionTask(waistTask, w_waist, 1, 0.0)


# POSTURE Task
print(robot.nv)
print(len(kp_posture))
postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture) # Proportional gain defined before (different for each joints)
postureTask.setKd(2.0 * kp_posture) # Derivative gain = 2 * kp
# Add the task with weight = 0.1, priority level = 1 (in cost function) and a transition duration = 0.0
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)

# CONTACTS 6D
# Definition of the foot geometry with respect to the ankle joints (which are the ones controlled)
contact_Point = np.ones((3, 4)) * lz
contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]

# The feet are the only bodies in contact in this experiment and their geometry defines the plane of contact
# between the robot and the environement -> it is a Contact6D

# To define a contact6D :
# We need the surface of contact (contact_point), the normal vector of contact (contactNormal along the z-axis)
# the friction parameter with the ground (mu = 0.3), the normal force bounds (fMin =1.0, fMax=1000.0)

# Right Foot
contactRF = tsid.Contact6d("contact_rfoot", robot, rf_frame_name, contact_Point, contactNormal, mu, fMin, fMax)
contactRF.setKp(kp_contact * np.ones(6)) # Proportional gain defined before = 30
contactRF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6)) # Derivative gain = 2 * sqrt(30)
# Reference position of the right ankle -> initial position
H_rf_ref = robot.position(data, model.getJointId(rf_frame_name))
contactRF.setReference(H_rf_ref)
# Add the contact to the HQP with weight = 0.1 for the force regularization task,
# and priority level = 0 (as real constraint) for the motion constraint
invdyn.addRigidContact(contactRF, w_forceRef)

# Left Foot
contactLF = tsid.Contact6d("contact_lfoot", robot, lf_frame_name, contact_Point, contactNormal, mu, fMin, fMax)
contactLF.setKp(kp_contact * np.ones(6)) # Proportional gain defined before = 30
contactLF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6)) # Derivative gain = 2 * sqrt(30)
# Reference position of the left ankle -> initial position
H_lf_ref = robot.position(data, model.getJointId(lf_frame_name))
contactLF.setReference(H_lf_ref)
# Add the contact to the HQP with weight = 0.1 for the force regularization task,
# and priority level = 0 (as real constraint) for the motion constraint
invdyn.addRigidContact(contactLF, w_forceRef)

# Set the reference trajectory of the tasks

com_ref = data.com[0] # Initial value of the CoM
trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
sampleCom = trajCom.computeNext() # Compute the first step of the trajectory from the initial value

q_ref = q[7:] # Initial value of the joints of the robot (in halfSitting position without the freeFlyer (6 first values))
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)

waist_ref = robot.position(data, model.getJointId('root_joint')) # Initial value of the waist (root_joint)
# Here the waist is defined as a 6d vector (position + orientation) so it is in the SE3 group (Lie group)
# Thus, the trajectory is not Euclidian but remains in the SE3 domain -> TrajectorySE3Constant
trajWaist = tsid.TrajectorySE3Constant("traj_waist", waist_ref)

# Initialisation of the Solver
# Use EiquadprogFast: dynamic matrix sizes (memory allocation performed only when resizing)
solver = tsid.SolverHQuadProgFast("qp solver")
# Resize the solver to fit the number of variables, equality and inequality constraints
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

# Initialisation of the plot variables which will be updated during the simulation loop
# These variables describe the behavior of the CoM of the robot (reference and real position, velocity and acceleration)
com_pos = np.full((3, N_SIMULATION), np.nan)
com_vel = np.full((3, N_SIMULATION), np.nan)
com_acc = np.full((3, N_SIMULATION), np.nan)

com_pos_ref = np.full((3, N_SIMULATION), np.nan)
com_vel_ref = np.full((3, N_SIMULATION), np.nan)
com_acc_ref = np.full((3, N_SIMULATION), np.nan)
com_acc_des = np.full((3, N_SIMULATION), np.nan)

# Parametes of the CoM sinusoid

offset  = robot.com(data)  # offset of the measured CoM
print(offset)
amp = np.array([0.01, 0.0, 0.0])  # amplitude function of 0.05 along the y axis
two_pi_f = 2 * np.pi * np.array([0.1, 0.0, 0.0])  # 2π function along the y axis with 0.5 amplitude
two_pi_f_amp = two_pi_f * amp  # 2π function times amplitude function
two_pi_f_squared_amp = two_pi_f * two_pi_f_amp  # 2π function times squared amplitude function

# Simulation loop
# At each time step compute the next desired positions of the tasks
# Set them as new references for each tasks

# The CoM trajectory is set with the sinusoid parameters:
# a sine for the position, a cosine (derivative of sine) for the velocity
# and a -sine (derivative of cosine) for the acceleration

# Compute the new problem data (HQP problem update)
# Solve the problem with the solver

# Get the forces and the accelerations computed by the solver
# Update the plot variables of the CoM
# Print the forces applied at each feet
# Print the tracking error of the CoM task and the norm of the velocity and acceleration needed to follow the
# reference trajectory

# Integrate the acceleration computed by the QP
# One simple Euler integration from acceleration to velocity
# One integration (velocity to position) with pinocchio to have the freeFlyer updated
# Display the result with gepetto viewer

for i in range(N_SIMULATION):
    time_start = tmp.time()
    print("--")
    sampleCom.value(offset + amp * np.sin(two_pi_f * t))
    sampleCom.derivative(two_pi_f_amp * np.cos(two_pi_f * t))
    sampleCom.second_derivative(-two_pi_f_squared_amp * np.sin(two_pi_f * t))
    comTask.setReference(sampleCom)

    sampleWaist = trajWaist.computeNext()
    # waistTask.setReference(sampleWaist)

    samplePosture = trajPosture.computeNext()
    postureTask.setReference(samplePosture)

    HQPData = invdyn.computeProblemData(t, q, v)
    print(HQPData)
    # if i == 0:
    #     HQPData.print_all()

    sol = solver.solve(HQPData)
    if sol.status != 0:
        print("QP problem could not be solved! Error code:", sol.status)
        break

    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)

    com_pos[:,i] = robot.com(invdyn.data())
    com_vel[:,i] = robot.com_vel(invdyn.data())
    com_acc[:,i] = comTask.getAcceleration(dv)
    com_pos_ref[:,i] = sampleCom.pos()
    com_vel_ref[:,i] = sampleCom.vel()
    com_acc_ref[:,i] = sampleCom.acc()
    com_acc_des[:,i] = comTask.getDesiredAcceleration

    if i % PRINT_N == 0:
        print("Time %.3f" % t)
        if invdyn.checkContact(contactRF.name, sol):
            f = invdyn.getContactForce(contactRF.name, sol)
            print("\tnormal force %s: %.1f" % (contactRF.name.ljust(20, '.'), contactRF.getNormalForce(f)))

        if invdyn.checkContact(contactLF.name, sol):
            f = invdyn.getContactForce(contactLF.name, sol)
            print("\tnormal force %s: %.1f" % (contactLF.name.ljust(20,  '.'), contactLF.getNormalForce(f)))

        print("\ttracking err %s: %.3f" % (comTask.name.ljust(20, '.'), norm(comTask.position_error, 2)))
        print("\t||v||: %.3f\t ||dv||: %.3f" % (norm(v, 2), norm(dv)))

    v_mean = v + 0.5 * dt * dv
    v += dt * dv
    q = pin.integrate(model, q, dt * v_mean)
    t += dt

    if i % DISPLAY_N == 0:
        viz.display(q)

    time_spent = tmp.time() - time_start
    if time_spent < dt:
        tmp.sleep(dt - time_spent)

# PLOT the result
time = np.arange(0.0, N_SIMULATION * dt, dt)

# Position tracking of the CoM along the x,y,z axis

(f, ax) = plut.create_empty_figure(3, 1, figsize=(10, 10))
for i in range(3):
    ax[i].plot(time, com_pos[i,:], label='CoM %i' % i)
    ax[i].plot(time, com_pos_ref[i,:], 'r:', label='CoM Ref %i' % i)
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM [m]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()

# Velocity tracking of the CoM along the x,y,z axis

(f, ax) = plut.create_empty_figure(3, 1, figsize=(10, 10))
for i in range(3):
    ax[i].plot(time, com_vel[i,:], label='CoM Vel %i' % i)
    ax[i].plot(time, com_vel_ref[i,:], 'r:', label='CoM Vel Ref %i' % i)
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM Vel [m/s]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()

# Acceleration tracking of the CoM along the x,y,z axis

(f, ax) = plut.create_empty_figure(3, 1, figsize=(10, 10))
for i in range(3):
    ax[i].plot(time, com_acc[i,:], label='CoM Acc %i' % i)
    ax[i].plot(time, com_acc_ref[i,:], 'r:', label='CoM Acc Ref %i' % i)
    ax[i].plot(time, com_acc_des[i,:], 'g--', label='CoM Acc Des %i' % i)
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM Acc [m/s^2]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

plt.show()
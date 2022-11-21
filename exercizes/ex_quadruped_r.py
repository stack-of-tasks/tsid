#!/usr/bin/env python3

import os
import tsid
import pinocchio as pin
import time
import numpy as np
import plot_utils as plut
import matplotlib.pyplot as plt

import os
import numpy as np
# import raisimpy as raisim
from RobotSimulation import RaiSim
import time

from whole_body_state_conversions import whole_body_controller_publisher

import rospy
from std_msgs.msg import String

rospy.init_node('talker', anonymous=True)

N_SIMULATION = 15000
dt = 0.001
sim = RaiSim(dt)

# raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
# world = raisim.World()
# world.setTimeStep(0.001)
# ground = world.addGround()

path = '/home/kian/catkin_ws/src/example-robot-data/robots'
urdf = path + '/anymal_raisim/urdf/anymal.urdf'
# Create the robot wrapper from the urdf, it will give the model of the robot and its data
robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
srdf = path + '/anymal_raisim/srdf/anymal.srdf'

# robot
# anymal = world.addArticulatedSystem(urdf)
# # launch raisim server
# server = raisim.RaisimServer(world)
# server.launchServer(8080)

sim.addRobot(urdf)

path = '/home/kian/catkin_ws/src/example-robot-data/robots'
urdf = path + '/anymal_b_simple_description/robots/anymal.urdf'
# # Create the robot wrapper from the urdf, it will give the model of the robot and its data
# robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
srdf = path + '/anymal_b_simple_description/srdf/anymal.srdf'


# Creation of the robot wrapper for gepetto viewer (graphical interface)
robot_display = pin.RobotWrapper.BuildFromURDF(urdf, [os.path.join(path, '../..')], pin.JointModelFreeFlyer())
Viewer = pin.visualize.MeshcatVisualizer
viz = Viewer(robot_display.model, robot_display.collision_model, robot_display.visual_model)
viz.initViewer(loadModel=True)

model = robot.model()
wbcp = whole_body_controller_publisher.WholeBodyControllerPublisher('/test/ing', model)
pin.loadReferenceConfigurations(model, srdf, False)
# Set the current configuration q to the robot configuration half_sitting
# q = model.referenceConfigurations['standing']
q = model.referenceConfigurations['standing']
v = np.zeros(robot.nv)

# Display the robot in Gepetto Viewer in the configuration q = halfSitting
#viz.display(q)
q[8] = q[8] + 0.001
# viz.display(q)

# anymal.setGeneralizedCoordinate(q)
sim.updateConfiguration(q)
# anymal.setPdGains(200 * np.ones([18]), np.ones([18]))
# anymal.setPdTarget(anymal_nominal_joint_config, np.zeros([18]))

rf_frame_name = "RH_KFE"        # right foot joint name
lf_frame_name = "LF_KFE"        # left foot joint name
rh_frame_name = "RH_KFE"        # right foot joint name
lh_frame_name = "LH_KFE"        # left foot joint name
contactNormal = np.array([0., 0., 1.])  # direction of the normal to the contact surface

t = 0

invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)

data = invdyn.data()

w_com = 1
kp_com = 20.0      # proportional gain of center of mass task
# COM Task
comTask = tsid.TaskComEquality("task-com", robot)
comTask.setKp(kp_com * np.ones(3)) # Proportional gain defined before = 20
comTask.setKd(2.0 * np.sqrt(kp_com) * np.ones(3)) # Derivative gain = 2 * sqrt(20)
# Add the task to the HQP with weight = 1.0, priority level = 0 (as constraint) and a transition duration = 0.0
invdyn.addMotionTask(comTask, w_com, 0, 0.0)

com_ref = data.com[0] # Initial value of the CoM
trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
sampleCom = trajCom.computeNext() # Compute the first step of the trajectory from the initial value

q_ref = q
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)
samplePosture = trajPosture.computeNext()

##################################################

lxp = 0.1      # foot length in positive x direction
lxn = 0.11     # foot length in negative x direction
lyp = 0.069    # foot length in positive y direction
lyn = 0.069    # foot length in negative y direction
lz = 0.107     # foot sole height with respect to ankle joint
mu = 0.3       # friction coefficient
fMin = 1.0     # minimum normal force
fMax = 1000.0  # maximum normal force

kp_contact = 30.0  # proportional gain of contact constraint
w_forceRef = 1e-3  # weight of force regularization task 1e-3

contact_Point = np.ones((3, 4)) * lz
contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]

contact_Point2 = np.ones((3, 4)) * (lz+0.1)
contact_Point2[0, :] = [-lxn, -lxn, lxp, lxp]
contact_Point2[1, :] = [-lyn, lyp, -lyn, lyp]
contactNormal2 = np.array([0., 0.1, 1.])  # direction of the normal to the contact surface

contactLF = tsid.ContactPoint("contact_lfoot", robot, lf_frame_name, contactNormal, mu, fMin, fMax)
contactLF.setKp(kp_contact * np.ones(3)) # Proportional gain defined before = 30
contactLF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(3)) # Derivative gain = 2 * sqrt(30)
# Reference position of the left ankle -> initial position
H_lf_ref = robot.position(data, model.getJointId(lf_frame_name))
contactLF.setReference(H_lf_ref)
# Add the contact to the HQP with weight = 0.1 for the force regularization task,
# and priority level = 0 (as real constraint) for the motion constraint
invdyn.addRigidContact(contactLF, w_forceRef)

contactRF = tsid.ContactPoint("contact_rfoot", robot, rf_frame_name, contactNormal, mu, fMin, fMax)
contactRF.setKp(kp_contact * np.ones(6)) # Proportional gain defined before = 30
contactRF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6)) # Derivative gain = 2 * sqrt(30)
# Reference position of the left ankle -> initial position
H_rf_ref = robot.position(data, model.getJointId(rf_frame_name))
contactRF.setReference(H_rf_ref)
# Add the contact to the HQP with weight = 0.1 for the force regularization task,
# and priority level = 0 (as real constraint) for the motion constraint
invdyn.addRigidContact(contactRF, 200)




# contactRF = tsid.Contact6d("contact_rfoot", robot, rf_frame_name, contact_Point, contactNormal, mu, fMin, fMax)
# contactRF.setKp(kp_contact * np.ones(6)) # Proportional gain defined before = 30
# contactRF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6)) # Derivative gain = 2 * sqrt(30)
# # Reference position of the left ankle -> initial position
# H_rf_ref = robot.position(data, model.getJointId(rf_frame_name))
# contactRF.setReference(H_rf_ref)
# # Add the contact to the HQP with weight = 0.1 for the force regularization task,
# # and priority level = 0 (as real constraint) for the motion constraint
# invdyn.addRigidContact(contactRF, 200)


# contactLH = tsid.Contact6d("contact_lh_foot", robot, lh_frame_name, contact_Point, contactNormal, mu, fMin, fMax)
# contactLH.setKp(kp_contact * np.ones(6)) # Proportional gain defined before = 30
# contactLH.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6)) # Derivative gain = 2 * sqrt(30)
# # Reference position of the left ankle -> initial position
# H_lh_ref = robot.position(data, model.getJointId(lh_frame_name))
# contactLH.setReference(H_lh_ref)
# # Add the contact to the HQP with weight = 0.1 for the force regularization task,
# # and priority level = 0 (as real constraint) for the motion constraint
# invdyn.addRigidContact(contactLH, w_forceRef)



# contactRH = tsid.Contact6d("contact_rh_foot", robot, rh_frame_name, contact_Point, contactNormal, mu, fMin, fMax)
# contactRH.setKp(kp_contact * np.ones(6)) # Proportional gain defined before = 30
# contactRH.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6)) # Derivative gain = 2 * sqrt(30)
# # Reference position of the left ankle -> initial position
# H_rh_ref = robot.position(data, model.getJointId(rh_frame_name))
# contactRH.setReference(H_rh_ref)
# # Add the contact to the HQP with weight = 0.1 for the force regularization task,
# # and priority level = 0 (as real constraint) for the motion constraint
# invdyn.addRigidContact(contactRH, w_forceRef)

#################################################

kp_posture = np.array(                       # proportional gain of joint posture task
[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]                              # head
)
w_posture = 10

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture) # Proportional gain defined before (different for each joints)
postureTask.setKd(2.0 * kp_posture) # Derivative gain = 2 * kp
# Add the task with weight = 0.1, priority level = 1 (in cost function) and a transition duration = 0.0
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)

#################################################################


# Initialisation of the Solver
# Use EiquadprogFast: dynamic matrix sizes (memory allocation performed only when resizing)
solver = tsid.SolverHQuadProgFast("qp solver")
# Resize the solver to fit the number of variables, equality and inequality constraints
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn) # THIS LINE COULD BE USEFUL!!!


offset = robot.com(data)  # offset of the measured CoM
amp = np.array([0.01, 0.01, 0])  # amplitude function of 0.05 along the y axis
two_pi_f = 2 * np.pi * np.array([0.5, 1, 0])  # 2π function along the y axis with 0.5 amplitude
two_pi_f_amp = two_pi_f * amp  # 2π function times amplitude function
two_pi_f_squared_amp = two_pi_f * two_pi_f_amp  # 2π function times squared amplitude function


q0 = q


com_pos = np.full((3, N_SIMULATION), np.nan)
com_pos_ref = np.full((3, N_SIMULATION), np.nan)

v_ref = np.zeros(len(q_ref))


for i in range(N_SIMULATION):
    # t = t + dt

    # server.integrateWorldThreadSafe()


    sampleCom.value(offset + amp * np.sin(two_pi_f * t))
    sampleCom.derivative(two_pi_f_amp * np.cos(two_pi_f * t))
    sampleCom.second_derivative(-two_pi_f_squared_amp * np.sin(two_pi_f * t))
    comTask.setReference(sampleCom)

    com_pos[:,i] = robot.com(invdyn.data())
    com_pos_ref[:,i] = offset + amp * np.sin(two_pi_f * t)


    # if i < 5000:
    #     q_ref[7] = q_ref[7] - 0.0001
    #     v_ref[8] = 0.001/dt
    #     # q_ref[8] = q_ref[8] - 0.0008
    # # else:
    # #     q_ref[7] = q_ref[7] - 0.005
    # #     # q_ref[8] = q_ref[8] - 0.005
    # # v_ref[0] =
    samplePosture.value(q_ref)
    postureTask.setReference(samplePosture)
    HQPData = invdyn.computeProblemData(t, q, v)
    # HQPData.print_all()

    sol = solver.solve(HQPData)
    if sol.status != 0:
        print("QP problem could not be solved! Error code:", sol.status)
        break
    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)

    v_mean = v + 0.5 * dt * dv
    v += dt * dv
    q = pin.integrate(model, q, dt * v_mean)
    t += dt

    wbcp.publish(t, q, q, v, v, tau, tau)

    sim.updateConfiguration(q)
    time.sleep(dt)
    print(i)
    # f = invdyn.getContactForce(contactLF.name, sol)
    # sim.printImpulse()

sim.kill()

time = np.arange(0.0, N_SIMULATION * dt, dt)

(f, ax) = plut.create_empty_figure(3, 1, figsize=(10, 10))

for i in range(3):
    ax[i].plot(time, com_pos[i,:], label='CoM %i' % i)
    ax[i].plot(time, com_pos_ref[i,:], 'r:', label='CoM Ref %i' % i)
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM [m]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)


plt.show()

import pinocchio as se3
import tsid
import numpy as np
from numpy.linalg import norm as norm
import os

import gepetto.corbaserver
import time
import commands

np.set_printoptions(precision=3, linewidth=200)

print "".center(100,'#')
print " Test Task Space Inverse Dynamics ".center(100, '#')
print "".center(100,'#'), '\n'

lxp = 0.14                          # foot length in positive x direction
lxn = 0.077                         # foot length in negative x direction
lyp = 0.069                         # foot length in positive y direction
lyn = 0.069                         # foot length in negative y direction
lz = 0.105                          # foot sole height with respect to ankle joint
mu = 0.3                            # friction coefficient
fMin = 5.0                          # minimum normal force
fMax = 1000.0                       # maximum normal force
rf_frame_name = "RAnkleRoll"        # right foot frame name
lf_frame_name = "LAnkleRoll"        # left foot frame name
contactNormal = np.matrix([0., 0., 1.]).T   # direction of the normal to the contact surface
w_com = 1.0                     # weight of center of mass task
w_posture = 1e-3                # weight of joint posture task
w_forceReg = 1e-5               # weight of force regularization task
w_RF = 1.0                      # weight of right foot motion task
kp_contact = 30.0               # proportional gain of contact constraint
kp_com = 30.0                   # proportional gain of center of mass task
kp_posture = 30.0               # proportional gain of joint posture task
kp_RF = 30.0                    # proportional gain of right foot motion task
REMOVE_CONTACT_N = 100          # remove right foot contact constraint after REMOVE_CONTACT_N time steps
CONTACT_TRANSITION_TIME = 1.0   # duration of the contact transition (to smoothly get a zero contact force before removing a contact constraint)
DELTA_COM_Y = 0.1               # distance between initial and desired center of mass position in y direction
DELTA_FOOT_Z = 0.1              # desired elevation of right foot in z direction
dt = 0.001                      # controller time step
PRINT_N = 500                   # print every PRINT_N time steps
DISPLAY_N = 25                  # update robot configuration in viwewer every DISPLAY_N time steps
N_SIMULATION = 4000             # number of time steps simulated

filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + '/../models/romeo'
urdf = path + '/urdf/romeo.urdf'
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)
srdf = path + '/srdf/romeo_collision.srdf'

# for gepetto viewer .. but Fix me!!
robot_display = se3.RobotWrapper.BuildFromURDF(urdf, [path, ], se3.JointModelFreeFlyer())

l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
if int(l[1]) == 0:
    os.system('gepetto-gui &')
time.sleep(1)
cl = gepetto.corbaserver.Client()
gui = cl.gui
robot_display.initDisplay(loadModel=True)

q = se3.getNeutralConfiguration(robot.model(), srdf, False)
q[2] += 0.84
v = np.matrix(np.zeros(robot.nv)).transpose()

robot_display.displayCollisions(False)
robot_display.displayVisuals(True)
robot_display.display(q)

assert robot.model().existFrame(rf_frame_name)
assert robot.model().existFrame(lf_frame_name)

t = 0.0                         # time
invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)
data = invdyn.data()
contact_Point = np.matrix(np.ones((3,4)) * lz)
contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]

contactRF =tsid.Contact6d("contact_rfoot", robot, rf_frame_name, contact_Point, contactNormal, mu, fMin, fMax)
contactRF.setKp(kp_contact * np.matrix(np.ones(6)).transpose())
contactRF.setKd(2.0 * np.sqrt(kp_contact) * np.matrix(np.ones(6)).transpose())
H_rf_ref = robot.position(data, robot.model().getJointId(rf_frame_name))
contactRF.setReference(H_rf_ref)
invdyn.addRigidContact(contactRF, w_forceReg)

contactLF =tsid.Contact6d("contact_lfoot", robot, lf_frame_name, contact_Point, contactNormal, mu, fMin, fMax)
contactLF.setKp(kp_contact * np.matrix(np.ones(6)).transpose())
contactLF.setKd(2.0 * np.sqrt(kp_contact) * np.matrix(np.ones(6)).transpose())
H_lf_ref = robot.position(data, robot.model().getJointId(lf_frame_name))
contactLF.setReference(H_lf_ref)
invdyn.addRigidContact(contactLF, w_forceReg)

comTask = tsid.TaskComEquality("task-com", robot)
comTask.setKp(kp_com * np.matrix(np.ones(3)).transpose())
comTask.setKd(2.0 * np.sqrt(kp_com) * np.matrix(np.ones(3)).transpose())
invdyn.addMotionTask(comTask, w_com, 1, 0.0)

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture * np.matrix(np.ones(robot.nv-6)).transpose())
postureTask.setKd(2.0 * np.sqrt(kp_posture) * np.matrix(np.ones(robot.nv-6)).transpose())
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)

rightFootTask = tsid.TaskSE3Equality("task-right-foot", robot, rf_frame_name)
rightFootTask.setKp(kp_RF * np.matrix(np.ones(6)).transpose())
rightFootTask.setKd(2.0 * np.sqrt(kp_com) * np.matrix(np.ones(6)).transpose())
invdyn.addMotionTask(rightFootTask, w_RF, 1, 0.0)

H_rf_ref.translation += np.matrix([0., 0., DELTA_FOOT_Z]).T
rightFootTraj = tsid.TrajectorySE3Constant("traj-right-foot", H_rf_ref)

com_ref = robot.com(data)
com_ref[1] += DELTA_COM_Y
trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)

q_ref = q[7:]
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)

solver = tsid.SolverHQuadProg("qp solver")
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

for i in range(0, N_SIMULATION):
    time_start = time.time()
    
    if i == REMOVE_CONTACT_N:
        print "\nTime %.3f Start breaking contact %s\n"%(t, contactRF.name)
        invdyn.removeRigidContact(contactRF.name, CONTACT_TRANSITION_TIME)

    sampleCom = trajCom.computeNext()
    comTask.setReference(sampleCom)
    samplePosture = trajPosture.computeNext()
    postureTask.setReference(samplePosture)
    sampleRightFoot = rightFootTraj.computeNext()
    rightFootTask.setReference(sampleRightFoot)

    HQPData = invdyn.computeProblemData(t, q, v)
    if i == 0:
        HQPData.print_all()

    sol = solver.solve(HQPData)
    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)

    if i%PRINT_N == 0:
        print "Time %.3f"%(t)
        if invdyn.checkContact(contactRF.name, sol):
            f = invdyn.getContactForce(contactRF.name, sol)
            print "\tnormal force %s: %.1f"%(contactRF.name.ljust(20,'.'),contactRF.getNormalForce(f))

        if invdyn.checkContact(contactLF.name, sol):
            f = invdyn.getContactForce(contactLF.name, sol)
            print "\tnormal force %s: %.1f"%(contactLF.name.ljust(20,'.'),contactLF.getNormalForce(f))

        print "\ttracking err %s: %.3f"%(comTask.name.ljust(20,'.'),       norm(comTask.position_error, 2))
        print "\ttracking err %s: %.3f"%(rightFootTask.name.ljust(20,'.'), norm(rightFootTask.position_error, 2))
        print "\t||v||: %.3f\t ||dv||: %.3f"%(norm(v, 2), norm(dv))

    v_mean = v + 0.5*dt*dv
    v += dt*dv
    q = se3.integrate(robot.model(), q, dt*v_mean)
    t += dt
    
    if i%DISPLAY_N == 0:
        robot_display.display(q)

    time_spent = time.time() - time_start
    if(time_spent < dt):
        time.sleep(dt-time_spent)
        
    assert norm(dv) < 1e6
    assert norm(v) < 1e6

print "\nFinal COM Position  ", robot.com(invdyn.data()).T
print "Desired COM Position", com_ref.T

import pinocchio as se3
import tsid
import numpy as np
from numpy.linalg import norm as norm
import os

import gepetto.corbaserver
import time
# import commands

np.set_printoptions(precision=3, linewidth=200, suppress=True)

print("".center(100,'#'))
print(" Test Task Space Inverse Dynamics ".center(100, '#'))
print("".center(100,'#'), '\n')

mu = 0.3                            # friction coefficient
fMin = 1.0                          # minimum normal force
fMax = 30.0                       # maximum normal force
contactNormal = np.matrix([0., 0., 1.]).T   # direction of the normal to the contact surface
w_com = 1.0                     # weight of center of mass task
w_forceRef = 1e-5               # weight of force regularization task
kp_contact = 100.0               # proportional gain of contact constraint
kp_com = 100.0                   # proportional gain of center of mass task
dt = 0.001                      # controller time step
N_SIMULATION = 1000             # number of time steps simulated

filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + '/../models'
urdf = path + '/quadruped/urdf/quadruped.urdf'
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)
srdf = path + '/srdf/romeo_collision.srdf'

print("Creating RobotWrapper")
# for gepetto viewer .. but Fix me!!
robot_display = se3.RobotWrapper(urdf, [path, ], se3.JointModelFreeFlyer())

# l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
# if int(l[1]) == 0:
#     os.system('gepetto-gui &')
time.sleep(1)
print("Connect to gepetto server.")

cl = gepetto.corbaserver.Client()
gui = cl.gui

robot_display.initDisplay(loadModel=True)

q = np.matrix(np.zeros(robot.nq)).T
q[2] = 0.5
q[0] = 0.1
q[6] = 1.
for i in range(4):
  q[7 + 2*i] = -0.8
  q[8 + 2*i] = 1.6
v = np.matrix(np.zeros(robot.nv)).transpose()

# Create the main invdyn formulation object.
t = 0.0
invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, True)
invdyn.computeProblemData(t, q, v)
data = invdyn.data()

robot.computeAllTerms(data, q, v)

# Place the robot onto the ground.
id_fl_contact = robot_display.model.getFrameId('FL_contact')
q[2] -= robot.framePosition(data, id_fl_contact).translation[2, 0]

contact_frames = [
  'BL_contact', 'BR_contact', 'FL_contact', 'FR_contact'
]
robot_display.displayCollisions(False)
robot_display.displayVisuals(True)
robot_display.display(q)

robot.computeAllTerms(data, q, v)

# Add task for the COM.
com_ref = robot.com(data)
comTask = tsid.TaskComEquality("task-com", robot)
comTask.setKp(kp_com * np.matrix(np.ones(3)).transpose())
comTask.setKd(2.0 * np.sqrt(kp_com) * np.matrix(np.ones(3)).transpose())
invdyn.addMotionTask(comTask, w_com, 1, 0.0)

trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
comTask.setReference(trajCom.computeNext())

# Add contact constraint for the point feets.
# HACK: Not taking the feet orientation into account for local frame right now.

task_contacts = []

for cframe in contact_frames:
  contact = tsid.ContactPoint("contact_" + cframe, robot, cframe, contactNormal, mu, fMin, fMax, w_forceRef)
  contact.setKp(kp_contact * np.matrix(np.ones(3)).transpose())
  contact.setKd(2.0 * np.sqrt(kp_contact) * np.matrix(np.ones(3)).transpose())
  contact.setReference(robot.framePosition(data, robot_display.model.getFrameId(cframe)))
  contact.useLocalFrame(False)
  invdyn.addRigidContact(contact, 1)

  task_contacts.append(contact)

# Add task to keep the robot at the current com position.


solver = tsid.SolverHQuadProg("qp solver")
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

print('COM start:', com_ref)

# for i in range(0, N_SIMULATION):
for i in range(0, N_SIMULATION):
  HQPData = invdyn.computeProblemData(t, q, v)

  sol = solver.solve(HQPData)
  tau = invdyn.getActuatorForces(sol)
  dv = invdyn.getAccelerations(sol)
  lam = invdyn.getContactForces(sol)

  # Minimal integrator.
  v_mean = v + 0.5*dt*dv
  v += dt*dv
  q = se3.integrate(robot.model(), q, dt*v_mean)
  t += dt

  com_ref[2] += 0.05 * dt
  trajCom.setReference(com_ref)
  comTask.setReference(trajCom.computeNext())

  robot_display.display(q)
  time.sleep(0.001)

robot.computeAllTerms(data, q, v)
com_final = robot.com(data)

print('COM ref:', com_ref)
print('COM final:', com_final)
print('COM error:', np.linalg.norm(com_final - com_ref))

import numpy as np
import numpy.matlib as matlib
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import plot_utils as plut
import time
import pinocchio as se3
import tsid
import gepetto.corbaserver
import commands
import os

import ur5_conf as conf

print "".center(conf.LINE_WIDTH,'#')
print " Joint Space Inverse Dynamics - Manipulator ".center(conf.LINE_WIDTH, '#')
print "".center(conf.LINE_WIDTH,'#'), '\n'

PLOT_JOINT_POS = 1
PLOT_JOINT_VEL = 1
PLOT_JOINT_ACC = 1
PLOT_TORQUES = 0
USE_VIEWER = 1

vector = se3.StdVec_StdString()
vector.extend(item for item in conf.path)
robot = tsid.RobotWrapper(conf.urdf, vector, False)
model = robot.model()

formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
q0 = conf.q0
v0 = np.matrix(np.zeros(robot.nv)).T
formulation.computeProblemData(0.0, q0, v0)
        
postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(conf.kp_posture * matlib.ones(robot.nv).T)
postureTask.setKd(2.0 * np.sqrt(conf.kp_posture) * matlib.ones(robot.nv).T)
formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)

trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q0)
postureTask.setReference(trajPosture.computeNext())

v_max = conf.v_max_scaling * model.velocityLimit
v_min = -v_max
#jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot, conf.dt)
#jointBoundsTask.setVelocityBounds(v_min, v_max)
#formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

if(USE_VIEWER):
    robot_display = se3.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path, ])
    l = commands.getstatusoutput("ps aux |grep 'gepetto-gui'|grep -v 'grep'|wc -l")
    if int(l[1]) == 0:
        os.system('gepetto-gui &')
    time.sleep(1)
    gepetto.corbaserver.Client()
    robot_display.initDisplay(loadModel=True)
    robot_display.displayCollisions(False)
    robot_display.displayVisuals(True)
    robot_display.display(q0)
    robot_display.viewer.gui.setCameraTransform(0, conf.CAMERA_TRANSFORM)

N = conf.N_SIMULATION
tau    = matlib.empty((robot.na, N))*nan
q      = matlib.empty((robot.nq, N+1))*nan
v      = matlib.empty((robot.nv, N+1))*nan
dv     = matlib.empty((robot.nv, N+1))*nan
q_ref  = matlib.empty((robot.nq, N))*nan
v_ref  = matlib.empty((robot.nv, N))*nan
dv_ref = matlib.empty((robot.nv, N))*nan
dv_des = matlib.empty((robot.nv, N))*nan
samplePosture = trajPosture.computeNext()

amp                  = np.matrix([0.2, 0.3, 0.4, 0.0, 0.0, 0.0]).T           # amplitude
phi                  = np.matrix([0.0, 0.5*np.pi, 0.0, 0.0, 0.0, 0.0]).T     # phase
two_pi_f             = 2*np.pi*np.matrix([1.0, 0.5, 0.3, 0.0, 0.0, 0.0]).T   # frequency (time 2 PI)
two_pi_f_amp         = np.multiply(two_pi_f, amp)
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

t = 0.0
dt = conf.dt
q[:,0], v[:,0] = q0, v0

for i in range(0, N):
    time_start = time.time()
    
    # set reference trajectory
    q_ref[:,i]  = q0 +  np.multiply(amp, matlib.sin(two_pi_f*t + phi))
    v_ref[:,i]  = np.multiply(two_pi_f_amp, matlib.cos(two_pi_f*t + phi))
    dv_ref[:,i] = np.multiply(two_pi_f_squared_amp, -matlib.sin(two_pi_f*t + phi))
    samplePosture.pos(q_ref[:,i])
    samplePosture.vel(v_ref[:,i])
    samplePosture.acc(dv_ref[:,i])
    postureTask.setReference(samplePosture)

    HQPData = formulation.computeProblemData(t, q[:,i], v[:,i])
    sol = solver.solve(HQPData)
    if(sol.status!=0):
        print "Time %.3f QP problem could not be solved! Error code:"%t, sol.status
        break
    
    tau[:,i] = formulation.getActuatorForces(sol)
    dv[:,i] = formulation.getAccelerations(sol)
    dv_des[:,i] = postureTask.getDesiredAcceleration

    if i%conf.PRINT_N == 0:
        print "Time %.3f"%(t)
        print "\ttracking err %s: %.3f"%(postureTask.name.ljust(20,'.'), norm(postureTask.position_error, 2))

    # numerical integration
    v_mean = v[:,i] + 0.5*dt*dv[:,i]
    v[:,i+1] = v[:,i] + dt*dv[:,i]
    q[:,i+1] = se3.integrate(model, q[:,i], dt*v_mean)
    t += conf.dt
    
    if i%conf.DISPLAY_N == 0: 
        robot_display.display(q[:,i])

    time_spent = time.time() - time_start
    if(time_spent < conf.dt): time.sleep(conf.dt-time_spent)

# PLOT STUFF
time = np.arange(0.0, N*conf.dt, conf.dt)

if(PLOT_JOINT_POS):    
    (f, ax) = plut.create_empty_figure(robot.nv/2,2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, q[i,:-1].A1, label='Joint pos '+str(i))
        ax[i].plot(time, q_ref[i,:].A1, '--', label='Joint ref pos '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Joint angles [rad]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)
        
if(PLOT_JOINT_VEL):    
    (f, ax) = plut.create_empty_figure(robot.nv/2,2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, v[i,:-1].A1, label='Joint vel '+str(i))
        ax[i].plot(time, v_ref[i,:].A1, '--', label='Joint ref vel '+str(i))
        ax[i].plot([time[0], time[-1]], 2*[v_min[i,0]], ':')
        ax[i].plot([time[0], time[-1]], 2*[v_max[i,0]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Joint velocity [rad/s]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)
        
if(PLOT_JOINT_ACC):    
    (f, ax) = plut.create_empty_figure(robot.nv/2,2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, dv[i,:-1].A1, label='Joint acc '+str(i))
        ax[i].plot(time, dv_ref[i,:].A1, '--', label='Joint ref acc '+str(i))
        ax[i].plot(time, dv_des[i,:].A1, ':', label='Joint des acc '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Joint acceleration [rad/s^2]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)
   
if(PLOT_TORQUES):    
    (f, ax) = plut.create_empty_figure(robot.nv/2,2)
    ax = ax.reshape(robot.nv)
    for i in range(robot.nv):
        ax[i].plot(time, tau[i,:].A1, label='Torque '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Torque [Nm]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)
        
plt.show()

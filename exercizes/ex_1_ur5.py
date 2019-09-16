import numpy as np
import numpy.matlib as matlib
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import plot_utils as plut
import time
from tsid_manipulator import TsidManipulator

#import ur5_conf as conf
import ur5_reaching_conf as conf

print "".center(conf.LINE_WIDTH,'#')
print " Task Space Inverse Dynamics - Manipulator ".center(conf.LINE_WIDTH, '#')
print "".center(conf.LINE_WIDTH,'#'), '\n'

PLOT_EE_POS = 1
PLOT_EE_VEL = 1
PLOT_EE_ACC = 0
PLOT_JOINT_VEL = 1
PLOT_TORQUES = 1

tsid = TsidManipulator(conf)

N = conf.N_SIMULATION
tau    = matlib.empty((tsid.robot.na, N))*nan
q      = matlib.empty((tsid.robot.nq, N+1))*nan
v      = matlib.empty((tsid.robot.nv, N+1))*nan
ee_pos = matlib.empty((3, N))*nan
ee_vel = matlib.empty((3, N))*nan
ee_acc = matlib.empty((3, N))*nan
ee_pos_ref = matlib.empty((3, N))*nan
ee_vel_ref = matlib.empty((3, N))*nan
ee_acc_ref = matlib.empty((3, N))*nan
ee_acc_des = matlib.empty((3, N))*nan # acc_des = acc_ref - Kp*pos_err - Kd*vel_err

sampleEE = tsid.trajEE.computeNext()
samplePosture = tsid.trajPosture.computeNext()

offset               = sampleEE.pos()
offset[:3,0]        += conf.offset
two_pi_f_amp         = np.multiply(conf.two_pi_f, conf.amp)
two_pi_f_squared_amp = np.multiply(conf.two_pi_f, two_pi_f_amp)

pEE = offset.copy()
vEE = matlib.zeros((6,1))
aEE = matlib.zeros((6,1))

tsid.gui.addSphere('world/ee', conf.SPHERE_RADIUS, conf.EE_SPHERE_COLOR)
tsid.gui.addSphere('world/ee_ref', conf.REF_SPHERE_RADIUS, conf.EE_REF_SPHERE_COLOR)

t = 0.0
q[:,0], v[:,0] = tsid.q, tsid.v

for i in range(0, N):
    time_start = time.time()
    
    pEE[:3,0] = offset[:3,0] +  np.multiply(conf.amp, matlib.sin(conf.two_pi_f*t + conf.phi))
    vEE[:3,0] = np.multiply(two_pi_f_amp, matlib.cos(conf.two_pi_f*t + conf.phi))
    aEE[:3,0] = np.multiply(two_pi_f_squared_amp, -matlib.sin(conf.two_pi_f*t + conf.phi))
    sampleEE.pos(pEE)
    sampleEE.vel(vEE)
    sampleEE.acc(aEE)
    tsid.eeTask.setReference(sampleEE)

    HQPData = tsid.formulation.computeProblemData(t, q[:,i], v[:,i])
    # if i == 0: HQPData.print_all()

    sol = tsid.solver.solve(HQPData)
    if(sol.status!=0):
        print "Time %.3f QP problem could not be solved! Error code:"%t, sol.status
        break
    
    tau[:,i] = tsid.formulation.getActuatorForces(sol)
    dv = tsid.formulation.getAccelerations(sol)
    
    ee_pos[:,i] = tsid.robot.framePosition(tsid.formulation.data(), tsid.EE).translation
    ee_vel[:,i] = tsid.robot.frameVelocityWorldOriented(tsid.formulation.data(), tsid.EE).linear
    ee_acc[:,i] = tsid.eeTask.getAcceleration(dv)[:3,0]
    ee_pos_ref[:,i] = sampleEE.pos()[:3,0]
    ee_vel_ref[:,i] = sampleEE.vel()[:3,0]
    ee_acc_ref[:,i] = sampleEE.acc()[:3,0]
    ee_acc_des[:,i] = tsid.eeTask.getDesiredAcceleration[:3,0]

    if i%conf.PRINT_N == 0:
        print "Time %.3f"%(t)
        print "\ttracking err %s: %.3f"%(tsid.eeTask.name.ljust(20,'.'), norm(tsid.eeTask.position_error, 2))

    q[:,i+1], v[:,i+1] = tsid.integrate_dv(q[:,i], v[:,i], dv, conf.dt)
    t += conf.dt
    
    if i%conf.DISPLAY_N == 0: 
        tsid.robot_display.display(q[:,i])
        tsid.gui.applyConfiguration('world/ee',     ee_pos[:,i].A1.tolist()+[0,0,0,1.])
        tsid.gui.applyConfiguration('world/ee_ref', ee_pos_ref[:,i].A1.tolist()+[0,0,0,1.])

    time_spent = time.time() - time_start
    if(time_spent < conf.dt): time.sleep(conf.dt-time_spent)

# PLOT STUFF
time = np.arange(0.0, N*conf.dt, conf.dt)

if(PLOT_EE_POS):
    (f, ax) = plut.create_empty_figure(3,1)
    for i in range(3):
        ax[i].plot(time, ee_pos[i,:].A1, label='EE '+str(i))
        ax[i].plot(time, ee_pos_ref[i,:].A1, 'r:', label='EE Ref '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE [m]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_EE_VEL):
    (f, ax) = plut.create_empty_figure(3,1)
    for i in range(3):
        ax[i].plot(time, ee_vel[i,:].A1, label='EE Vel '+str(i))
        ax[i].plot(time, ee_vel_ref[i,:].A1, 'r:', label='EE Vel Ref '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE Vel [m/s]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_EE_ACC):    
    (f, ax) = plut.create_empty_figure(3,1)
    for i in range(3):
        ax[i].plot(time, ee_acc[i,:].A1, label='EE Acc '+str(i))
        ax[i].plot(time, ee_acc_ref[i,:].A1, 'r:', label='EE Acc Ref '+str(i))
        ax[i].plot(time, ee_acc_des[i,:].A1, 'g--', label='EE Acc Des '+str(i))
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('EE Acc [m/s^2]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_TORQUES):    
    (f, ax) = plut.create_empty_figure(tsid.robot.nv/2,2)
    ax = ax.reshape(tsid.robot.nv)
    for i in range(tsid.robot.nv):
        ax[i].plot(time, tau[i,:].A1, label='Torque '+str(i))
        ax[i].plot([time[0], time[-1]], 2*[tsid.tau_min[i,0]], ':')
        ax[i].plot([time[0], time[-1]], 2*[tsid.tau_max[i,0]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Torque [Nm]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if(PLOT_JOINT_VEL):    
    (f, ax) = plut.create_empty_figure(tsid.robot.nv/2,2)
    ax = ax.reshape(tsid.robot.nv)
    for i in range(tsid.robot.nv):
        ax[i].plot(time, v[i,:-1].A1, label='Joint vel '+str(i))
        ax[i].plot([time[0], time[-1]], 2*[tsid.v_min[i,0]], ':')
        ax[i].plot([time[0], time[-1]], 2*[tsid.v_max[i,0]], ':')
        ax[i].set_xlabel('Time [s]')
        ax[i].set_ylabel('Joint velocity [rad/s]')
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)
        
plt.show()

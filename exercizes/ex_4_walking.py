import numpy as np
import numpy.matlib as matlib
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import plot_utils as plut
import time
import ex_4_conf as conf
from tsid_biped import TsidBiped

print "".center(conf.LINE_WIDTH,'#')
print " Test Walking ".center(conf.LINE_WIDTH, '#')
print "".center(conf.LINE_WIDTH,'#'), '\n'

data = np.load(conf.DATA_FILE)

tsid = TsidBiped(conf)

N = data['com'].shape[1]
com_pos = matlib.empty((3, N))*nan
com_vel = matlib.empty((3, N))*nan
com_acc = matlib.empty((3, N))*nan
f_RF = matlib.zeros((6, N))
f_LF = matlib.zeros((6, N))
cop_RF = matlib.zeros((2, N))
cop_LF = matlib.zeros((2, N))

foot_steps = np.asmatrix(data['foot_steps'])
contact_phase = data['contact_phase']
com_pos_ref = np.asmatrix(data['com'])
com_vel_ref = np.asmatrix(data['dcom'])
com_acc_ref = np.asmatrix(data['ddcom'])*1.0
com_acc_des = matlib.empty((3, N))*nan # acc_des = acc_ref - Kp*pos_err - Kd*vel_err

x_rf   = tsid.get_placement_RF().translation
offset = matlib.zeros((3,1))
offset[:2,0] = x_rf[:2,0] - foot_steps[0,:].T #tsid.robot.com(tsid.formulation.data())
for i in range(N):
    com_pos_ref[:,i] += offset
#    foot_steps[:,i] += offset[:2,0]

sampleCom = tsid.trajCom.computeNext()

t = 0.0
q, v = tsid.q, tsid.v

for i in range(-2000, N):
    time_start = time.time()
    
    if i==0:
        print "Starting to walk (remove contact left foot)"
        tsid.remove_contact_LF()
    elif i>0:
        if contact_phase[i] != contact_phase[i-1]:
            print "Changing contact phase from %s to %s"%(contact_phase[i-1], contact_phase[i])
            if contact_phase[i] == 'left':
                tsid.add_contact_LF()
                tsid.remove_contact_RF()
            else:
                tsid.add_contact_RF()
                tsid.remove_contact_LF()
    
    if i<0:
        sampleCom.pos(com_pos_ref[:,0])
    else:
        sampleCom.pos(com_pos_ref[:,i])
        sampleCom.vel(com_vel_ref[:,i])
        sampleCom.acc(com_acc_ref[:,i])

    tsid.comTask.setReference(sampleCom)
    tsid.rightFootTask.setReference(tsid.trajRF.computeNext())
    tsid.leftFootTask.setReference(tsid.trajLF.computeNext())
    
    HQPData = tsid.formulation.computeProblemData(t, q, v)

    sol = tsid.solver.solve(HQPData)
    if(sol.status!=0):
        print "QP problem could not be solved! Error code:", sol.status
        break
    if norm(v,2)>20.0:
        print "Time %.3f Velocities are too high, stop everything!"%(t), norm(v)
        break
    
    tau = tsid.formulation.getActuatorForces(sol)
    dv = tsid.formulation.getAccelerations(sol)
    
    if i>=0:
        com_pos[:,i] = tsid.robot.com(tsid.formulation.data())
        com_vel[:,i] = tsid.robot.com_vel(tsid.formulation.data())
        com_acc[:,i] = tsid.comTask.getAcceleration(dv)
        com_acc_des[:,i] = tsid.comTask.getDesiredAcceleration
        if tsid.formulation.checkContact(tsid.contactRF.name, sol):
            T_RF = tsid.contactRF.getForceGeneratorMatrix
            f_RF[:,i] = T_RF * tsid.formulation.getContactForce(tsid.contactRF.name, sol)
            if(f_RF[2,i]>1e-3): 
                cop_RF[0,i] = f_RF[4,i] / f_RF[2,i]
                cop_RF[1,i] = -f_RF[3,i] / f_RF[2,i]
        if tsid.formulation.checkContact(tsid.contactLF.name, sol):
            T_LF = tsid.contactRF.getForceGeneratorMatrix
            f_LF[:,i] = T_LF * tsid.formulation.getContactForce(tsid.contactLF.name, sol)
            if(f_LF[2,i]>1e-3): 
                cop_LF[0,i] = f_LF[4,i] / f_LF[2,i]
                cop_LF[1,i] = -f_LF[3,i] / f_LF[2,i]

    if i%conf.PRINT_N == 0:
        print "Time %.3f"%(t)
        if tsid.formulation.checkContact(tsid.contactRF.name, sol):
            print "\tnormal force %s: %.1f"%(tsid.contactRF.name.ljust(20,'.'), f_RF[2,i])

        if tsid.formulation.checkContact(tsid.contactLF.name, sol):
            print "\tnormal force %s: %.1f"%(tsid.contactLF.name.ljust(20,'.'), f_LF[2,i])

        print "\ttracking err %s: %.3f"%(tsid.comTask.name.ljust(20,'.'), norm(tsid.comTask.position_error, 2))
        print "\t||v||: %.3f\t ||dv||: %.3f"%(norm(v, 2), norm(dv))

    q, v = tsid.integrate_dv(q, v, dv, conf.dt)
    t += conf.dt
    
    if i%conf.DISPLAY_N == 0: tsid.robot_display.display(q)

    time_spent = time.time() - time_start
    if(time_spent < conf.dt): time.sleep(conf.dt-time_spent)
    
# PLOT STUFF
time = np.arange(0.0, N*conf.dt, conf.dt)

(f, ax) = plut.create_empty_figure(3,1)
for i in range(3):
    ax[i].plot(time, com_pos[i,:].A1, label='CoM '+str(i))
    ax[i].plot(time, com_pos_ref[i,:].A1, 'r:', label='CoM Ref '+str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM [m]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

(f, ax) = plut.create_empty_figure(3,1)
for i in range(3):
    ax[i].plot(time, com_vel[i,:].A1, label='CoM Vel '+str(i))
    ax[i].plot(time, com_vel_ref[i,:].A1, 'r:', label='CoM Vel Ref '+str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM Vel [m/s]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)
    
(f, ax) = plut.create_empty_figure(3,1)
for i in range(3):
    ax[i].plot(time, com_acc[i,:].A1, label='CoM Acc '+str(i))
    ax[i].plot(time, com_acc_ref[i,:].A1, 'r:', label='CoM Acc Ref '+str(i))
    ax[i].plot(time, com_acc_des[i,:].A1, 'g--', label='CoM Acc Des '+str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoM Acc [m/s^2]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)

(f, ax) = plut.create_empty_figure(2,1)
for i in range(2):
    ax[i].plot(time, cop_LF[i,:].A1, label='CoP LF '+str(i))
    ax[i].plot(time, cop_RF[i,:].A1, label='CoP RF '+str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('CoP [m]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)
    
(f, ax) = plut.create_empty_figure(3,2)
ax = ax.reshape((6))
for i in range(6):
    ax[i].plot(time, f_LF[i,:].A1, label='Force LF '+str(i))
    ax[i].plot(time, f_RF[i,:].A1, label='Force RF '+str(i))
    ax[i].set_xlabel('Time [s]')
    ax[i].set_ylabel('Force [N/Nm]')
    leg = ax[i].legend()
    leg.get_frame().set_alpha(0.5)
    
plt.show()

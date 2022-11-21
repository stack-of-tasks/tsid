import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import plot_utils as plut
import time
import anymal_conf as conf
import pinocchio as pin
from tsid_quadruped import TsidQuadruped


print("".center(conf.LINE_WIDTH, '#'))
print(" TSID - Quadruped ".center(conf.LINE_WIDTH, '#'))
print("".center(conf.LINE_WIDTH, '#'), '\n')

tsid = TsidQuadruped(conf, conf.viewer)

N = conf.N_SIMULATION
com_pos = np.empty((3, N)) * nan
com_vel = np.empty((3, N)) * nan
com_acc = np.empty((3, N)) * nan

com_pos_ref = np.empty((3, N)) * nan
com_vel_ref = np.empty((3, N)) * nan
com_acc_ref = np.empty((3, N)) * nan
com_acc_des = np.empty((3, N)) * nan  # acc_des = acc_ref - Kp*pos_err - Kd*vel_err


offset = tsid.robot.com(tsid.formulation.data())
amp = np.array([0.0, 0.0, 0.0])
two_pi_f = 2 * np.pi * np.array([0.0, 0.0, 0.0])
two_pi_f_amp = two_pi_f * amp
two_pi_f_squared_amp = two_pi_f * two_pi_f_amp

sampleCom = tsid.trajCom.computeNext()
samplePosture = tsid.trajPosture.computeNext()

t = 0.0
q, v = tsid.q, tsid.v

for i in range(0, N):
    time_start = time.time()

    sampleCom.value(offset + amp * np.sin(two_pi_f * t))
    sampleCom.derivative(two_pi_f_amp * np.cos(two_pi_f * t))
    sampleCom.second_derivative(-two_pi_f_squared_amp * np.sin(two_pi_f * t))

    tsid.comTask.setReference(sampleCom)
    tsid.postureTask.setReference(samplePosture)

    HQPData = tsid.formulation.computeProblemData(t, q, v)
    # if i == 0: HQPData.print_all()

    sol = tsid.solver.solve(HQPData)
    if (sol.status != 0):
        print("QP problem could not be solved! Error code:", sol.status)
        break

    tau = tsid.formulation.getActuatorForces(sol)
    dv = tsid.formulation.getAccelerations(sol)

    com_pos[:, i] = tsid.robot.com(tsid.formulation.data())
    com_vel[:, i] = tsid.robot.com_vel(tsid.formulation.data())
    com_acc[:, i] = tsid.comTask.getAcceleration(dv)
    com_pos_ref[:, i] = sampleCom.value()
    com_vel_ref[:, i] = sampleCom.derivative()
    com_acc_ref[:, i] = sampleCom.second_derivative()
    com_acc_des[:, i] = tsid.comTask.getDesiredAcceleration

    if i % conf.PRINT_N == 0:
        print("Time %.3f" % (t))
        if tsid.formulation.checkContact(tsid.contactRF.name, sol):
            f = tsid.formulation.getContactForce(tsid.contactRF.name, sol)
            print("\tnormal force %s: %.1f" % (tsid.contactRF.name.ljust(20, '.'), tsid.contactRF.getNormalForce(f)))

        if tsid.formulation.checkContact(tsid.contactLF.name, sol):
            f = tsid.formulation.getContactForce(tsid.contactLF.name, sol)
            print("\tnormal force %s: %.1f" % (tsid.contactLF.name.ljust(20, '.'), tsid.contactLF.getNormalForce(f)))

        print("\ttracking err %s: %.3f" % (tsid.comTask.name.ljust(20, '.'), norm(tsid.comTask.position_error, 2)))
        print("\t||v||: %.3f\t ||dv||: %.3f" % (norm(v, 2), norm(dv)))

    q, v = tsid.integrate_dv(q, v, dv, conf.dt)
    t += conf.dt

    if i % conf.DISPLAY_N == 0: tsid.display(q)

    time_spent = time.time() - time_start
    if (time_spent < conf.dt): time.sleep(conf.dt - time_spent)

# PLOT STUFF
time = np.arange(0.0, N * conf.dt, conf.dt)

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

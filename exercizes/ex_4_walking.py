import time

import ex_4_conf as conf
import matplotlib.pyplot as plt
import numpy as np
import plot_utils as plut
import tsid
from numpy import nan
from numpy.linalg import norm as norm
from tsid_biped import TsidBiped

print("".center(conf.LINE_WIDTH, "#"))
print(" Test Walking ".center(conf.LINE_WIDTH, "#"))
print("".center(conf.LINE_WIDTH, "#"), "\n")


USE_EIQUADPROG = 1
USE_PROXQP = 0
USE_OSQP = 0
VERBOSE = 0

PLOT_COM = 1
PLOT_COP = 1
PLOT_FOOT_TRAJ = 0
PLOT_TORQUES = 0
PLOT_JOINT_VEL = 0

try:
    data = np.load(conf.DATA_FILE_TSID)
except FileNotFoundError as e:
    print(
        "To run this walking example, you need to first execute ex_4_plan_LIPM_romeo.py and ex_4_LIPM_to_TSID."
    )
    raise e

tsid_biped = TsidBiped(conf, conf.viewer)

# overwrite the default solver
if USE_EIQUADPROG:
    print("Using eiquadprog")
    tsid_biped.solver = tsid.SolverHQuadProgFast("qp solver")

elif USE_PROXQP:
    print("Using proxqp")
    tsid_biped.solver = tsid.SolverProxQP("qp solver")
    tsid_biped.solver.set_epsilon_absolute(1e-5)
    tsid_biped.solver.set_maximum_iterations(int(1e6))
    tsid_biped.solver.set_verbose(VERBOSE)

elif USE_OSQP:
    print("Using osqp")
    tsid_biped.solver = tsid.SolverOSQP("qp solver")
    tsid_biped.solver.set_epsilon_absolute(1e-8)
    tsid_biped.solver.set_maximum_iterations(int(1e6))
    tsid_biped.solver.set_verbose(VERBOSE)

else:
    print("Please specify which solver to use.")
    exit()

tsid_biped.solver.resize(
    tsid_biped.formulation.nVar, tsid_biped.formulation.nEq, tsid_biped.formulation.nIn
)

N = data["com"].shape[1]
N_pre = int(conf.T_pre / conf.dt)
N_post = int(conf.T_post / conf.dt)

com_pos = np.empty((3, N + N_post)) * nan
com_vel = np.empty((3, N + N_post)) * nan
com_acc = np.empty((3, N + N_post)) * nan
x_LF = np.empty((3, N + N_post)) * nan
dx_LF = np.empty((3, N + N_post)) * nan
ddx_LF = np.empty((3, N + N_post)) * nan
ddx_LF_des = np.empty((3, N + N_post)) * nan
x_RF = np.empty((3, N + N_post)) * nan
dx_RF = np.empty((3, N + N_post)) * nan
ddx_RF = np.empty((3, N + N_post)) * nan
ddx_RF_des = np.empty((3, N + N_post)) * nan
f_RF = np.zeros((6, N + N_post))
f_LF = np.zeros((6, N + N_post))
cop_RF = np.zeros((2, N + N_post))
cop_LF = np.zeros((2, N + N_post))
tau = np.zeros((tsid_biped.robot.na, N + N_post))
q_log = np.zeros((tsid_biped.robot.nq, N + N_post))
v_log = np.zeros((tsid_biped.robot.nv, N + N_post))

contact_phase = data["contact_phase"]
com_pos_ref = np.asarray(data["com"])
com_vel_ref = np.asarray(data["dcom"])
com_acc_ref = np.asarray(data["ddcom"])
x_RF_ref = np.asarray(data["x_RF"])
dx_RF_ref = np.asarray(data["dx_RF"])
ddx_RF_ref = np.asarray(data["ddx_RF"])
x_LF_ref = np.asarray(data["x_LF"])
dx_LF_ref = np.asarray(data["dx_LF"])
ddx_LF_ref = np.asarray(data["ddx_LF"])
cop_ref = np.asarray(data["cop"])
com_acc_des = (
    np.empty((3, N + N_post)) * nan
)  # acc_des = acc_ref - Kp*pos_err - Kd*vel_err

x_rf = tsid_biped.get_placement_RF().translation
offset = x_rf - x_RF_ref[:, 0]
for i in range(N):
    com_pos_ref[:, i] += offset + np.array([0.0, 0.0, 0.0])
    x_RF_ref[:, i] += offset
    x_LF_ref[:, i] += offset

t = -conf.T_pre
q, v = tsid_biped.q, tsid_biped.v

qp_data_list = []
c = 0
q_list = []

input("Press enter to start")
for i in range(-N_pre, N + N_post):
    time_start = time.time()

    if i == 0:
        print("Starting to walk (remove contact left foot)")
        tsid_biped.remove_contact_LF()
    elif i > 0 and i < N - 1:
        if contact_phase[i] != contact_phase[i - 1]:
            print(
                f"Time {t:.3f} Changing contact phase from {contact_phase[i - 1]} to {contact_phase[i]}"
            )
            if contact_phase[i] == "left":
                tsid_biped.add_contact_LF()
                tsid_biped.remove_contact_RF()
            else:
                tsid_biped.add_contact_RF()
                tsid_biped.remove_contact_LF()

    if i < 0:
        tsid_biped.set_com_ref(
            com_pos_ref[:, 0], 0 * com_vel_ref[:, 0], 0 * com_acc_ref[:, 0]
        )
    elif i < N:
        tsid_biped.set_com_ref(com_pos_ref[:, i], com_vel_ref[:, i], com_acc_ref[:, i])
        tsid_biped.set_LF_3d_ref(x_LF_ref[:, i], dx_LF_ref[:, i], ddx_LF_ref[:, i])
        tsid_biped.set_RF_3d_ref(x_RF_ref[:, i], dx_RF_ref[:, i], ddx_RF_ref[:, i])

    HQPData = tsid_biped.formulation.computeProblemData(t, q, v)
    sol = tsid_biped.solver.solve(HQPData)

    if sol.status != 0:
        print("QP problem could not be solved! Error code:", sol.status)
        break
    if norm(v, 2) > 40.0:
        print(f"Time {t:.3f} Velocities are too high, stop everything!", norm(v))
        break

    if i > 0:
        q_log[:, i] = q
        v_log[:, i] = v
        tau[:, i] = tsid_biped.formulation.getActuatorForces(sol)
    dv = tsid_biped.formulation.getAccelerations(sol)

    if i >= 0:
        com_pos[:, i] = tsid_biped.robot.com(tsid_biped.formulation.data())
        com_vel[:, i] = tsid_biped.robot.com_vel(tsid_biped.formulation.data())
        com_acc[:, i] = tsid_biped.comTask.getAcceleration(dv)
        com_acc_des[:, i] = tsid_biped.comTask.getDesiredAcceleration
        x_LF[:, i], dx_LF[:, i], ddx_LF[:, i] = tsid_biped.get_LF_3d_pos_vel_acc(dv)
        if not tsid_biped.contact_LF_active:
            ddx_LF_des[:, i] = tsid_biped.leftFootTask.getDesiredAcceleration[:3]
        x_RF[:, i], dx_RF[:, i], ddx_RF[:, i] = tsid_biped.get_RF_3d_pos_vel_acc(dv)
        if not tsid_biped.contact_RF_active:
            ddx_RF_des[:, i] = tsid_biped.rightFootTask.getDesiredAcceleration[:3]

        if tsid_biped.formulation.checkContact(tsid_biped.contactRF.name, sol):
            T_RF = tsid_biped.contactRF.getForceGeneratorMatrix
            f_RF[:, i] = T_RF.dot(
                tsid_biped.formulation.getContactForce(tsid_biped.contactRF.name, sol)
            )
            if f_RF[2, i] > 1e-3:
                cop_RF[0, i] = f_RF[4, i] / f_RF[2, i]
                cop_RF[1, i] = -f_RF[3, i] / f_RF[2, i]
        if tsid_biped.formulation.checkContact(tsid_biped.contactLF.name, sol):
            T_LF = tsid_biped.contactLF.getForceGeneratorMatrix
            f_LF[:, i] = T_LF.dot(
                tsid_biped.formulation.getContactForce(tsid_biped.contactLF.name, sol)
            )
            if f_LF[2, i] > 1e-3:
                cop_LF[0, i] = f_LF[4, i] / f_LF[2, i]
                cop_LF[1, i] = -f_LF[3, i] / f_LF[2, i]

    if i % conf.PRINT_N == 0:
        print(f"Time {t:.3f}")
        if (
            tsid_biped.formulation.checkContact(tsid_biped.contactRF.name, sol)
            and i >= 0
        ):
            print(
                "\tnormal force {}: {:.1f}".format(
                    tsid_biped.contactRF.name.ljust(20, "."), f_RF[2, i]
                )
            )

        if (
            tsid_biped.formulation.checkContact(tsid_biped.contactLF.name, sol)
            and i >= 0
        ):
            print(
                "\tnormal force {}: {:.1f}".format(
                    tsid_biped.contactLF.name.ljust(20, "."), f_LF[2, i]
                )
            )

        print(
            "\ttracking err {}: {:.3f}".format(
                tsid_biped.comTask.name.ljust(20, "."),
                norm(tsid_biped.comTask.position_error, 2),
            )
        )
        print(f"\t||v||: {norm(v, 2):.3f}\t ||dv||: {norm(dv):.3f}")

    q, v = tsid_biped.integrate_dv(q, v, dv, conf.dt)
    t += conf.dt

    q_list.append(q)

    if i % conf.DISPLAY_N == 0:
        tsid_biped.display(q)

    time_spent = time.time() - time_start
    if time_spent < conf.dt:
        time.sleep(conf.dt - time_spent)

while True:
    replay = input("Play video again? [Y]/n: ") or "Y"
    if replay.lower() == "y":
        for q in q_list:
            tsid_biped.display(q)
    else:
        break
# PLOT STUFF
time = np.arange(0.0, (N + N_post) * conf.dt, conf.dt)

if PLOT_COM:
    (f, ax) = plut.create_empty_figure(3, 1)
    for i in range(3):
        ax[i].plot(time, com_pos[i, :], label="CoM " + str(i))
        ax[i].plot(time[:N], com_pos_ref[i, :], "r:", label="CoM Ref " + str(i))
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoM [m]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

    (f, ax) = plut.create_empty_figure(3, 1)
    for i in range(3):
        ax[i].plot(time, com_vel[i, :], label="CoM Vel " + str(i))
        ax[i].plot(time[:N], com_vel_ref[i, :], "r:", label="CoM Vel Ref " + str(i))
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoM Vel [m/s]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

    (f, ax) = plut.create_empty_figure(3, 1)
    for i in range(3):
        ax[i].plot(time, com_acc[i, :], label="CoM Acc " + str(i))
        ax[i].plot(time[:N], com_acc_ref[i, :], "r:", label="CoM Acc Ref " + str(i))
        ax[i].plot(time, com_acc_des[i, :], "g--", label="CoM Acc Des " + str(i))
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoM Acc [m/s^2]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)

if PLOT_COP:
    (f, ax) = plut.create_empty_figure(2, 1)
    for i in range(2):
        ax[i].plot(time, cop_LF[i, :], label="CoP LF " + str(i))
        ax[i].plot(time, cop_RF[i, :], label="CoP RF " + str(i))
        #        ax[i].plot(time[:N], cop_ref[i,:], label='CoP ref '+str(i))
        if i == 0:
            ax[i].plot(
                [time[0], time[-1]],
                [-conf.lxn, -conf.lxn],
                ":",
                label="CoP Lim " + str(i),
            )
            ax[i].plot(
                [time[0], time[-1]],
                [conf.lxp, conf.lxp],
                ":",
                label="CoP Lim " + str(i),
            )
        elif i == 1:
            ax[i].plot(
                [time[0], time[-1]],
                [-conf.lyn, -conf.lyn],
                ":",
                label="CoP Lim " + str(i),
            )
            ax[i].plot(
                [time[0], time[-1]],
                [conf.lyp, conf.lyp],
                ":",
                label="CoP Lim " + str(i),
            )
        ax[i].set_xlabel("Time [s]")
        ax[i].set_ylabel("CoP [m]")
        leg = ax[i].legend()
        leg.get_frame().set_alpha(0.5)


# (f, ax) = plut.create_empty_figure(3,2)
# ax = ax.reshape((6))
# for i in range(6):
#    ax[i].plot(time, f_LF[i,:], label='Force LF '+str(i))
#    ax[i].plot(time, f_RF[i,:], label='Force RF '+str(i))
#    ax[i].set_xlabel('Time [s]')
#    ax[i].set_ylabel('Force [N/Nm]')
#    leg = ax[i].legend()
#    leg.get_frame().set_alpha(0.5)

if PLOT_FOOT_TRAJ:
    for i in range(3):
        plt.figure()
        plt.plot(time, x_RF[i, :], label="x RF " + str(i))
        plt.plot(time[:N], x_RF_ref[i, :], ":", label="x RF ref " + str(i))
        plt.plot(time, x_LF[i, :], label="x LF " + str(i))
        plt.plot(time[:N], x_LF_ref[i, :], ":", label="x LF ref " + str(i))
        plt.legend()

    # for i in range(3):
    #    plt.figure()
    #    plt.plot(time, dx_RF[i,:], label='dx RF '+str(i))
    #    plt.plot(time[:N], dx_RF_ref[i,:], ':', label='dx RF ref '+str(i))
    #    plt.plot(time, dx_LF[i,:], label='dx LF '+str(i))
    #    plt.plot(time[:N], dx_LF_ref[i,:], ':', label='dx LF ref '+str(i))
    #    plt.legend()
    #
    # for i in range(3):
    #    plt.figure()
    #    plt.plot(time, ddx_RF[i,:], label='ddx RF '+str(i))
    #    plt.plot(time[:N], ddx_RF_ref[i,:], ':', label='ddx RF ref '+str(i))
    #    plt.plot(time, ddx_RF_des[i,:], '--', label='ddx RF des '+str(i))
    #    plt.plot(time, ddx_LF[i,:], label='ddx LF '+str(i))
    #    plt.plot(time[:N], ddx_LF_ref[i,:], ':', label='ddx LF ref '+str(i))
    #    plt.plot(time, ddx_LF_des[i,:], '--', label='ddx LF des '+str(i))
    #    plt.legend()

if PLOT_TORQUES:
    plt.figure()
    for i in range(tsid_biped.robot.na):
        tau_normalized = (
            2
            * (tau[i, :] - tsid_biped.tau_min[i])
            / (tsid_biped.tau_max[i] - tsid_biped.tau_min[i])
            - 1
        )
        # plot torques only for joints that reached 50% of max torque
        if np.max(np.abs(tau_normalized)) > 0.5:
            plt.plot(
                time, tau_normalized, alpha=0.5, label=tsid_biped.model.names[i + 2]
            )
    plt.plot([time[0], time[-1]], 2 * [-1.0], ":")
    plt.plot([time[0], time[-1]], 2 * [1.0], ":")
    plt.gca().set_xlabel("Time [s]")
    plt.gca().set_ylabel("Normalized Torque")
    leg = plt.legend()
    leg.get_frame().set_alpha(0.5)

if PLOT_JOINT_VEL:
    plt.figure()
    for i in range(tsid_biped.robot.na):
        v_normalized = (
            2
            * (v_log[6 + i, :] - tsid_biped.v_min[i])
            / (tsid_biped.v_max[i] - tsid_biped.v_min[i])
            - 1
        )
        # plot v only for joints that reached 50% of max v
        if np.max(np.abs(v_normalized)) > 0.5:
            plt.plot(time, v_normalized, alpha=0.5, label=tsid_biped.model.names[i + 2])
    plt.plot([time[0], time[-1]], 2 * [-1.0], ":")
    plt.plot([time[0], time[-1]], 2 * [1.0], ":")
    plt.gca().set_xlabel("Time [s]")
    plt.gca().set_ylabel("Normalized Joint Vel")
    leg = plt.legend()
#    leg.get_frame().set_alpha(0.5)

plt.show()

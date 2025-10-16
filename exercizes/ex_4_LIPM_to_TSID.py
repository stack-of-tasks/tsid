import ex_4_conf as conf
import LMPC_walking.second_order.plot_utils as plot_utils
import matplotlib.pyplot as plt
import numpy as np
from LMPC_walking.second_order.LIPM_to_whole_body import (
    compute_foot_traj,
    interpolate_lipm_traj,
)

# import ex_4_long_conf as conf


# READ COM-COP TRAJECTORIES COMPUTED WITH LIPM MODEL
data = np.load(conf.DATA_FILE_LIPM)
com_state_x = data["com_state_x"]
com_state_y = data["com_state_y"]
cop_ref = data["cop_ref"]
cop_x = data["cop_x"]
cop_y = data["cop_y"]
foot_steps = data["foot_steps"]

# INTERPOLATE WITH TIME STEP OF CONTROLLER (TSID)
dt_ctrl = conf.dt  # time step used by TSID
com, dcom, ddcom, cop, contact_phase, foot_steps_ctrl = interpolate_lipm_traj(
    conf.T_step,
    conf.nb_steps,
    conf.dt_mpc,
    dt_ctrl,
    conf.h,
    conf.g,
    com_state_x,
    com_state_y,
    cop_ref,
    cop_x,
    cop_y,
)

# COMPUTE TRAJECTORIES FOR FEET
# number of time steps for traj-opt
N = conf.nb_steps * round(conf.T_step / conf.dt_mpc)
N_ctrl = int((N * conf.dt_mpc) / dt_ctrl)  # number of time steps for TSID
foot_steps_RF = foot_steps[::2, :]  # assume first foot step corresponds to right foot
x_RF, dx_RF, ddx_RF = compute_foot_traj(
    foot_steps_RF, N_ctrl, dt_ctrl, conf.T_step, conf.step_height, "stance"
)
foot_steps_LF = foot_steps[1::2, :]
x_LF, dx_LF, ddx_LF = compute_foot_traj(
    foot_steps_LF, N_ctrl, dt_ctrl, conf.T_step, conf.step_height, "swing"
)

# SAVE COMPUTED TRAJECTORIES IN NPY FILE FOR TSID
np.savez(
    conf.DATA_FILE_TSID,
    com=com,
    dcom=dcom,
    ddcom=ddcom,
    x_RF=x_RF,
    dx_RF=dx_RF,
    ddx_RF=ddx_RF,
    x_LF=x_LF,
    dx_LF=dx_LF,
    ddx_LF=ddx_LF,
    contact_phase=contact_phase,
    cop=cop,
)

# PLOT STUFF
time_ctrl = np.arange(0, round(N_ctrl * dt_ctrl, 2), dt_ctrl)

for i in range(3):
    plt.figure()
    plt.plot(time_ctrl, x_RF[i, :-1], label="x RF " + str(i))
    plt.plot(time_ctrl, x_LF[i, :-1], label="x LF " + str(i))
    plt.legend()

# for i in range(2):
#    plt.figure()
#    plt.plot(time_ctrl, dx_RF[i,:-1], label='dx RF '+str(i))
#    plt.plot(time_ctrl, dx_LF[i,:-1], label='dx LF '+str(i))
#    plt.legend()
#
# for i in range(2):
#    plt.figure()
#    plt.plot(time_ctrl, ddx_RF[i,:-1], label='ddx RF '+str(i))
#    plt.plot(time_ctrl, ddx_LF[i,:-1], label='ddx LF '+str(i))
#    plt.legend()

time = np.arange(0, round(N * conf.dt_mpc, 2), conf.dt_mpc)
for i in range(2):
    plt.figure()
    plt.plot(time_ctrl, cop[i, :-1], label="CoP")
    #    plt.plot(time_ctrl, foot_steps_ctrl[i,:-1], label='Foot step')
    plt.plot(time_ctrl, com[i, :-1], "g", label="CoM")
    if i == 0:
        plt.plot(time, com_state_x[:-1, 0], ":", label="CoM TO")
    else:
        plt.plot(time, com_state_y[:-1, 0], ":", label="CoM TO")
    plt.legend()

# for i in range(2):
#    plt.figure()
#    plt.plot(time_ctrl, dcom[i,:-1], label='CoM vel')
#    vel_fd = (com[i,1:] - com[i,:-1]) / dt_ctrl
#    plt.plot(time_ctrl, vel_fd, ':', label='CoM vel fin-diff')
#    plt.legend()
#
# for i in range(2):
#    plt.figure()
#    plt.plot(time_ctrl, ddcom[i,:-1], label='CoM acc')
#    acc_fd = (dcom[i,1:] - dcom[i,:-1]) / dt_ctrl
#    plt.plot(time_ctrl, acc_fd, ':', label='CoM acc fin-diff')
#    plt.legend()

foot_length = conf.lxn + conf.lxp  # foot size in the x-direction
foot_width = conf.lyn + conf.lyp  # foot size in the y-direciton
plot_utils.plot_xy(
    time_ctrl,
    N_ctrl,
    foot_length,
    foot_width,
    foot_steps_ctrl.T,
    cop[0, :],
    cop[1, :],
    com[0, :].reshape((N_ctrl + 1, 1)),
    com[1, :].reshape((N_ctrl + 1, 1)),
)
plt.plot(
    com_state_x[:, 0],
    com_state_y[:, 0],
    "r* ",
    markersize=15,
)
plt.gca().set_xlim([-0.2, 0.4])
plt.gca().set_ylim([-0.3, 0.3])

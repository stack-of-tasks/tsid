import copy
import os
import time
import numpy as np
import pinocchio as pin
from numpy.linalg import norm

import tsid
import sys
sys.path.append('../../exercizes/')
import romeo_conf as conf
from tsid_biped import TsidBiped

print("")
print("Test Task Energy")
print("")

tsid_biped = TsidBiped(conf)

# Energy Task
energyTask = tsid.TaskEnergy("task-energy", tsid_biped.robot, conf.dt)
energyTask.set_E_tank(1.0)

N = conf.N_SIMULATION
t = 0.0
q, v = tsid_biped.q, tsid_biped.v
offset     = tsid_biped.robot.com(tsid_biped.formulation.data())
amp        = np.array([0.0, 0.05, 0.0])
two_pi_f             = 2*np.pi*np.array([0.0, 0.5, 0.0])
two_pi_f_amp         = two_pi_f * amp
two_pi_f_squared_amp = two_pi_f * two_pi_f_amp
energy_tank_fall_to_min = False
error_com_convergence = False
error_solver = False

for i in range(0, N):
    time_start = time.time()
    if i == 1:
        tsid_biped.formulation.addEnergyTask(energyTask, 1, 0, 0.0) 
        print("Energy Task Added ! ")

    sampleCom = tsid_biped.trajCom.computeNext()
    sampleCom.pos(offset + amp * np.sin(two_pi_f*t))
    sampleCom.vel( two_pi_f_amp * np.cos(two_pi_f*t))
    sampleCom.acc(-two_pi_f_squared_amp * np.sin(two_pi_f*t))

    tsid_biped.comTask.setReference(sampleCom)
    samplePosture = tsid_biped.trajPosture.computeNext()
    tsid_biped.postureTask.setReference(samplePosture)

    HQPData = tsid_biped.formulation.computeProblemData(t, q, v)

    sol = tsid_biped.solver.solve(HQPData)
    if(sol.status!=0):
        print("QP problem could not be solved! Error code:", sol.status)
        error_solver = True
        break
    if energyTask.get_E_tank < 0.1:
        energy_tank_fall_to_min = True
        break
    if norm(tsid_biped.comTask.position_error, 2) > 0.05:
        error_com_convergence = True
        break
    
    tau = tsid_biped.formulation.getActuatorForces(sol)
    dv = tsid_biped.formulation.getAccelerations(sol)

    if i%400 == 0:
        print("Time %.3f"%(t))
        if tsid_biped.formulation.checkContact(tsid_biped.contactRF.name, sol):
            f = tsid_biped.formulation.getContactForce(tsid_biped.contactRF.name, sol)
            print("\tnormal force %s: %.1f"%(tsid_biped.contactRF.name.ljust(20,'.'), tsid_biped.contactRF.getNormalForce(f)))

        if tsid_biped.formulation.checkContact(tsid_biped.contactLF.name, sol):
            f = tsid_biped.formulation.getContactForce(tsid_biped.contactLF.name, sol)
            print("\tnormal force %s: %.1f"%(tsid_biped.contactLF.name.ljust(20,'.'), tsid_biped.contactLF.getNormalForce(f)))

        print("\ttracking err %s: %.3f"%(tsid_biped.comTask.name.ljust(20,'.'), norm(tsid_biped.comTask.position_error, 2)))
        print("\t||v||: %.3f\t ||dv||: %.3f"%(norm(v, 2), norm(dv)))
        print("\tEnergy tank value: %.3f"%(energyTask.get_E_tank))

    q, v = tsid_biped.integrate_dv(q, v, dv, conf.dt)
    t += conf.dt
    
    if i%conf.DISPLAY_N == 0: tsid_biped.display(q)

    time_spent = time.time() - time_start
    if(time_spent < conf.dt): time.sleep(conf.dt-time_spent)

if not (energy_tank_fall_to_min) and not (error_com_convergence) and not (error_solver):
    print("All test is done")
else:
    print("Error with the Energy Tank or the CoM convergence or no solution found !")

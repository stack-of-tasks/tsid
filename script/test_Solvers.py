import pinocchio as se3
import tsid

import numpy as np
import copy

print ""
print "Test Solvers"
print ""

EPS = 1e-3
nTest = 100
n = 60
neq = 36
nin = 40
damping = 1e-10

NORMAL_DISTR_VAR = 10.0
GRADIENT_PERTURBATION_VARIANCE = 1e-2
HESSIAN_PERTURBATION_VARIANCE = 1e-1
MARGIN_PERC = 1e-3

print "Gonna perform", nTest, "tests with", n, "variables, ", neq, "equalities", nin, "inequalities"

solver = tsid.SolverHQuadProg("qp solver")
solver.resize(n, neq, nin)

HQPData = tsid.HQPData()
A1 = np.matrix(np.random.rand(n, n)) + 0.001 * np.matrix(np.eye(n))
b1 = np.matrix(np.random.rand(n)).transpose()
cost = tsid.ConstraintEquality("c1", A1, b1)

x = np.linalg.inv(A1) * b1
A_in = np.matrix(np.random.rand(nin, n))
A_lb = np.matrix(np.random.rand(nin)).transpose() * NORMAL_DISTR_VAR
A_ub = np.matrix(np.random.rand(nin)).transpose() * NORMAL_DISTR_VAR
constrVal = A_in * x

for i in range(0, nin):	
    if A_ub[i] <= A_lb[i]:
	A_ub[i] = A_lb[i] + MARGIN_PERC * np.abs(A_lb[i])
        A_lb[i] = A_lb[i] - MARGIN_PERC * np.abs(A_lb[i]) 

    if constrVal[i]> A_ub[i]:
        A_ub[i] = constrVal[i] + MARGIN_PERC * np.abs(constrVal[i])
    elif constrVal[i] < A_lb[i]:
        A_lb[i] = constrVal[i] - MARGIN_PERC * np.abs(constrVal[i])

in_const = tsid.ConstraintInequality("ini1", A_in, A_lb, A_ub)
A_eq = np.matrix(np.random.rand(neq, n))
b_eq = A_eq*x
eq_const = tsid.ConstraintEquality("eq1", A_eq, b_eq)

const1 = tsid.ConstraintLevel()
const1.append(1.0, eq_const)
const1.append(1.0, in_const)
print "check constraint level #0"
const1.print_all()

const2= tsid.ConstraintLevel()
const2.append(1.0, cost)
print "check constraint level #1"
const2.print_all()

HQPData.append(const1)
HQPData.append(const2)
print "Check HQP DATA"
HQPData.print_all()

gradientPerturbations = []
hessianPerturbations = []
for i in range (0, nTest):
    gradientPerturbations.append(np.matrix(np.random.rand(n) * GRADIENT_PERTURBATION_VARIANCE).transpose())
    hessianPerturbations.append(np.matrix(np.random.rand(n,n) * HESSIAN_PERTURBATION_VARIANCE))

for i in range (0, nTest):
    cost.setMatrix(cost.matrix + hessianPerturbations[i])
    cost.setVector(cost.vector + gradientPerturbations[i])

    HQPoutput = solver.solve(HQPData)

    assert np.linalg.norm(A_eq * HQPoutput.x - b_eq, 2) < EPS
    #assert (A_in * HQPoutput.x <= A_ub + EPS).all()
    #assert (A_in * HQPoutput.x > A_lb - EPS).all()
   

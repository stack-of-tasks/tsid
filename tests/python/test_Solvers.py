import numpy as np
import tsid

print("")
print("Test Solvers")
print("")

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

print(
    "Gonna perform",
    nTest,
    "tests with",
    n,
    "variables, ",
    neq,
    "equalities",
    nin,
    "inequalities",
)

solver_list = []

solver_eiquadprog = tsid.SolverHQuadProg("eiquadprog solver")
solver_eiquadprog.resize(n, neq, nin)
solver_list.append(("eiquadprog", solver_eiquadprog))
print("Adding eiquadprog to list of solvers to test")

try:
    solver_proxqp = tsid.SolverProxQP("proxqp solver")
    solver_proxqp.resize(n, neq, nin)
    solver_list.append(("proxqp", solver_proxqp))
    print("Adding proxqp to list of solvers to test")

except AttributeError:
    pass

try:
    solver_osqp = tsid.SolverOSQP("osqp solver")
    solver_osqp.resize(n, neq, nin)
    solver_list.append(("osqp", solver_osqp))
    print("Adding osqp to list of solvers to test")

except AttributeError:
    pass

HQPData = tsid.HQPData()
A1 = np.random.rand(n, n) + 0.001 * np.eye(n)
b1 = np.random.rand(n)
cost = tsid.ConstraintEquality("c1", A1, b1)

x = np.linalg.inv(A1).dot(b1)
A_in = np.random.rand(nin, n)
A_lb = np.random.rand(nin) * NORMAL_DISTR_VAR
A_ub = np.random.rand(nin) * NORMAL_DISTR_VAR
constrVal = A_in.dot(x)

for i in range(0, nin):
    if A_ub[i] <= A_lb[i]:
        A_ub[i] = A_lb[i] + MARGIN_PERC * np.abs(A_lb[i])
        A_lb[i] = A_lb[i] - MARGIN_PERC * np.abs(A_lb[i])

    if constrVal[i] > A_ub[i]:
        A_ub[i] = constrVal[i] + MARGIN_PERC * np.abs(constrVal[i])
    elif constrVal[i] < A_lb[i]:
        A_lb[i] = constrVal[i] - MARGIN_PERC * np.abs(constrVal[i])

in_const = tsid.ConstraintInequality("ini1", A_in, A_lb, A_ub)
A_eq = np.random.rand(neq, n)
b_eq = A_eq.dot(x)
eq_const = tsid.ConstraintEquality("eq1", A_eq, b_eq)

const1 = tsid.ConstraintLevel()
const1.append(1.0, eq_const)
const1.append(1.0, in_const)
print("check constraint level #0")
const1.print_all()

const2 = tsid.ConstraintLevel()
const2.append(1.0, cost)
print("check constraint level #1")
const2.print_all()

HQPData.append(const1)
HQPData.append(const2)
print("Check HQP DATA")
HQPData.print_all()

gradientPerturbations = []
hessianPerturbations = []
for i in range(0, nTest):
    gradientPerturbations.append(np.random.rand(n) * GRADIENT_PERTURBATION_VARIANCE)
    hessianPerturbations.append(np.random.rand(n, n) * HESSIAN_PERTURBATION_VARIANCE)

for name, solver in solver_list:
    print(f"Using {name}")
    for i in range(0, nTest):
        cost.setMatrix(cost.matrix + hessianPerturbations[i])
        cost.setVector(cost.vector + gradientPerturbations[i])

        HQPoutput = solver.solve(HQPData)

        assert np.linalg.norm(A_eq.dot(HQPoutput.x) - b_eq, 2) < EPS
        assert (A_in.dot(HQPoutput.x) <= A_ub + EPS).all()
        assert (A_in.dot(HQPoutput.x) > A_lb - EPS).all()
    print("-> succesful")

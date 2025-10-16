//
// Copyright (c) 2017 CNRS
//

#include <tsid/solvers/solver-HQP-factory.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-fast.hpp>

#ifdef TSID_QPMAD_FOUND
#include <tsid/solvers/solver-HQP-qpmad.hpp>
#endif

#ifdef TSID_WITH_PROXSUITE
#include <tsid/solvers/solver-proxqp.hpp>
#endif

#ifdef TSID_WITH_OSQP
#include <tsid/solvers/solver-osqp.hpp>
#endif

#ifdef QPOASES_FOUND
#include <tsid/solvers/solver-HQP-qpoases.hh>
#endif

namespace tsid {
namespace solvers {

SolverHQPBase* SolverHQPFactory::createNewSolver(const SolverHQP solverType,
                                                 const std::string& name) {
  if (solverType == SOLVER_HQP_EIQUADPROG) return new SolverHQuadProg(name);

  if (solverType == SOLVER_HQP_EIQUADPROG_FAST)
    return new SolverHQuadProgFast(name);

#ifdef TSID_QPMAD_FOUND
  if (solverType == SOLVER_HQP_QPMAD) return new SolverHQpmad(name);
#endif

#ifdef TSID_WITH_PROXSUITE
  if (solverType == SOLVER_HQP_PROXQP) return new SolverProxQP(name);
#endif

#ifdef TSID_WITH_OSQP
  if (solverType == SOLVER_HQP_OSQP) return new SolverOSQP(name);
#endif

#ifdef QPOASES_FOUND
  if (solverType == SOLVER_HQP_QPOASES) return new Solver_HQP_qpoases(name);
#endif

  PINOCCHIO_CHECK_INPUT_ARGUMENT(false, "Specified solver type not recognized");
  return NULL;
}

}  // namespace solvers
}  // namespace tsid

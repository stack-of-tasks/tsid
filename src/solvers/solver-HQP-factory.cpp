//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
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

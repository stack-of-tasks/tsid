//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/solvers/solver-HQP-eiquadprog.hpp"

namespace tsid {
namespace python {
void exposeSolverHQuadProg() {
  SolverHQuadProgPythonVisitor<tsid::solvers::SolverHQuadProg>::expose(
      "SolverHQuadProg");
  SolverHQuadProgPythonVisitor<tsid::solvers::SolverHQuadProgFast>::expose(
      "SolverHQuadProgFast");
}

void exposeSolverProxQP() {
#ifdef TSID_WITH_PROXSUITE
  SolverProxQPPythonVisitor<tsid::solvers::SolverProxQP>::expose(
      "SolverProxQP");
#endif
}

void exposeSolverOSQP() {
#ifdef TSID_WITH_OSQP
  SolverOSQPPythonVisitor<tsid::solvers::SolverOSQP>::expose("SolverOSQP");
#endif
}
}  // namespace python
}  // namespace tsid

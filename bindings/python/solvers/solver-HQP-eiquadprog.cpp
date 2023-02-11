//
// Copyright (c) 2018 CNRS
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

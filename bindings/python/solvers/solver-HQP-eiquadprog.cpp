
#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/solvers/solver-HQP-eiquadprog.hpp"

namespace tsid
{
  namespace python
  {
    void exposeSolverHQuadProg()
    {
      SolverHQuadProgPythonVisitor<tsid::solvers::SolverHQuadProg>::expose("SolverHQuadProg");
    }
  }
}

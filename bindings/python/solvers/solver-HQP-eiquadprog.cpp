
#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/solvers/solver-HQP-eiquadprog.hpp"

namespace tsid
{
  namespace python
  {
    void exposeSolverQuadProg()
    {
      SolverQuadProgPythonVisitor<tsid::solvers::SolverHQuadProg>::expose("SolverQuadProg");
    }
  }
}

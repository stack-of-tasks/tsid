#ifndef __tsid_python_expose_solvers_hpp__
#define __tsid_python_expose_solvers_hpp__


#include "tsid/bindings/python/solvers/solver-HQP-eiquadprog.hpp"
#include "tsid/bindings/python/solvers/HQPData.hpp"
#include "tsid/bindings/python/solvers/HQPOutput.hpp"
namespace tsid
{
  namespace python
  {
    void exposeSolverQuadProg();
    void exposeConstriantLevel();
    void exposeHQPData();
    void exposeHQPOutput();
    inline void exposeSolvers()
    {
      exposeSolverQuadProg();
      exposeConstriantLevel();
      exposeHQPData();
      exposeHQPOutput();
    }
    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_solvers_hpp__


#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/solvers/HQPOutput.hpp"

namespace tsid
{
  namespace python
  {
    void exposeHQPOutput()
    {
      HQPOutputPythonVisitor<solvers::HQPOutput>::expose("HQPOutput");
    }
  }
}

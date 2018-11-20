
#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/solvers/HQPData.hpp"

namespace tsid
{
  namespace python
  {
    void exposeConstriantLevel()
    {
      ConstPythonVisitor<ConstraintLevels>::expose("ConstraintLevel");
    }
    void exposeHQPData()
    {
      HQPPythonVisitor<HQPDatas>::expose("HQPData");
    }
  }
}

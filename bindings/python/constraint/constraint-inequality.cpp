
#include "tsid/bindings/python/constraint/constraint-inequality.hpp"
#include "tsid/bindings/python/constraint/expose-constraints.hpp"

namespace tsid
{
  namespace python
  {
    void exposeConstraintInequality()
    {
      ConstraintIneqPythonVisitor<tsid::math::ConstraintInequality>::expose("ConstraintInequality");
    }
  }
}

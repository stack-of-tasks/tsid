
#include "tsid/bindings/python/constraint/constraint-bound.hpp"
#include "tsid/bindings/python/constraint/expose-constraints.hpp"

namespace tsid
{
  namespace python
  {
    void exposeConstraintBound()
    {
      ConstraintPythonVisitor<tsid::math::ConstraintBound>::expose("ConstraintBound");
    }
  }
}

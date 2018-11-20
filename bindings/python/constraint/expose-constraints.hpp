#ifndef __tsid_python_expose_constraint_bound_hpp__
#define __tsid_python_expose_constraint_bound_hpp__

#include "tsid/bindings/python/constraint/constraint-bound.hpp"
#include "tsid/bindings/python/constraint/constraint-equality.hpp"
#include "tsid/bindings/python/constraint/constraint-inequality.hpp"
namespace tsid
{
  namespace python
  {
    void exposeConstraintBound();
    void exposeConstraintEquality();
    void exposeConstraintInequality();
    inline void exposeConstraints()
    {
      exposeConstraintBound();
      exposeConstraintEquality();
      exposeConstraintInequality();
    }
    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_constraint_bound_hpp__

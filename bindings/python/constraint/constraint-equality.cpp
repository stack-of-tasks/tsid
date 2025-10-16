//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/constraint/constraint-equality.hpp"
#include "tsid/bindings/python/constraint/expose-constraints.hpp"

namespace tsid {
namespace python {
void exposeConstraintEquality() {
  ConstraintEqPythonVisitor<tsid::math::ConstraintEquality>::expose(
      "ConstraintEquality");
}
}  // namespace python
}  // namespace tsid

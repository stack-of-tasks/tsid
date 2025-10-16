//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/trajectories/trajectory-se3.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"

namespace tsid {
namespace python {
void exposeTrajectorySE3Constant() {
  TrajectorySE3ConstantPythonVisitor<
      tsid::trajectories::TrajectorySE3Constant>::
      expose("TrajectorySE3Constant");
}
}  // namespace python
}  // namespace tsid

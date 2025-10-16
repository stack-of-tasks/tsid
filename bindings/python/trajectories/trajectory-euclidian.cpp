//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/trajectories/trajectory-euclidian.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"

namespace tsid {
namespace python {
void exposeTrajectoryEuclidianConstant() {
  TrajectoryEuclidianConstantPythonVisitor<
      tsid::trajectories::TrajectoryEuclidianConstant>::
      expose("TrajectoryEuclidianConstant");
}
}  // namespace python
}  // namespace tsid

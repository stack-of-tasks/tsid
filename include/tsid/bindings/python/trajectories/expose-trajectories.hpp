//
// Copyright (c) 2018 CNRS
//

#ifndef __tsid_python_expose_trajectories_hpp__
#define __tsid_python_expose_trajectories_hpp__

#include "tsid/bindings/python/trajectories/trajectory-se3.hpp"
#include "tsid/bindings/python/trajectories/trajectory-euclidian.hpp"
#include "tsid/bindings/python/trajectories/trajectory-base.hpp"

namespace tsid {
namespace python {
void exposeTrajectorySE3Constant();
void exposeTrajectoryEuclidianConstant();
void exposeTrajectorySample();

inline void exposeTrajectories() {
  exposeTrajectorySE3Constant();
  exposeTrajectoryEuclidianConstant();
  exposeTrajectorySample();
}
}  // namespace python
}  // namespace tsid
#endif  // ifndef __tsid_python_expose_trajectories_hpp__

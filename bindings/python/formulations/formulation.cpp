//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/formulations/expose-formulations.hpp"
#include "tsid/bindings/python/formulations/formulation.hpp"

namespace tsid {
namespace python {
void exposeInverseDynamicsFormulationAccForce() {
  InvDynPythonVisitor<tsid::InverseDynamicsFormulationAccForce>::expose(
      "InverseDynamicsFormulationAccForce");
}
}  // namespace python
}  // namespace tsid

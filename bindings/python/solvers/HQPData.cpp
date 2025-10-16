//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/solvers/HQPData.hpp"

namespace tsid {
namespace python {
void exposeConstraintLevel() {
  ConstPythonVisitor<ConstraintLevels>::expose("ConstraintLevel");
}
void exposeHQPData() {
  typedef solvers::QPDataBaseTpl<double> QPDataBase;
  typedef solvers::QPDataTpl<double> QPData;
  typedef solvers::QPDataQuadProgTpl<double> QPDataQuadProg;

  HQPPythonVisitor<HQPDatas>::expose("HQPData");

  // expose QP data structures
  bp::class_<QPDataBase>("QPDataBase")
      .def_readonly("H", &QPDataBase::H, "Cost matrix")
      .def_readonly("g", &QPDataBase::g)
      .def_readonly("CE", &QPDataBase::CE, "Equality constraint matrix")
      .def_readonly("ce0", &QPDataBase::ce0);

  bp::class_<QPData, bp::bases<QPDataBase>>("QPData")
      .def_readonly("CI", &QPData::CI, "Inequality constraint matrix")
      .def_readonly("lb", &QPData::ci_lb, "Inequality constraint lower bound")
      .def_readonly("ub", &QPData::ci_ub, "Inequality constraint upper bound");

  bp::class_<QPDataQuadProg, bp::bases<QPDataBase>>("QPDataQuadProg")
      .def_readonly("CI", &QPDataQuadProg::CI,
                    "Inequality constraint matrix (unilateral)")
      .def_readonly(
          "ci0", &QPDataQuadProg::ci0,
          "Inequality constraint vector (stacked lower and upper bounds)");
}
}  // namespace python
}  // namespace tsid

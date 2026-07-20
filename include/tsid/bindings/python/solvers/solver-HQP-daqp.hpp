//
// Copyright (c) 2026 CNRS
//

#ifndef __tsid_python_solver_daqp_hpp__
#define __tsid_python_solver_daqp_hpp__

#include "tsid/bindings/python/fwd.hpp"
#include "tsid/bindings/python/solvers/HQPData.hpp"
#include "tsid/solvers/solver-HQP-daqp.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename Solver>
struct SolverDAQPPythonVisitor
    : public bp::def_visitor<SolverDAQPPythonVisitor<Solver>> {
  template <class PyClass>
  void visit(PyClass& cl) const {
    cl.def(bp::init<const std::string&>(bp::arg("name")))
        .def("resize", &SolverDAQPPythonVisitor::resize,
             bp::args("n", "neq", "nin"))
        .def("solve", &SolverDAQPPythonVisitor::solve, bp::args("HQPData"))
        .add_property("ObjVal", &Solver::getObjectiveValue,
                      "Return the objective value.");
  }

  static void resize(Solver& self, unsigned int n, unsigned int neq,
                     unsigned int nin) {
    self.resize(n, neq, nin);
  }

  static solvers::HQPOutput solve(Solver& self, HQPDatas& data) {
    return self.solve(data.get());
  }

  static void expose(const std::string& className) {
    bp::class_<Solver>(className.c_str(), "DAQP hierarchical QP solver.",
                       bp::no_init)
        .def(SolverDAQPPythonVisitor<Solver>());
  }
};

}  // namespace python
}  // namespace tsid

#endif  // __tsid_python_solver_daqp_hpp__

//
// Copyright (c) 2022 INRIA
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __tsid_python_solver_osqp_hpp__
#define __tsid_python_solver_osqp_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/solvers/solver-osqp.hpp"

#include "tsid/solvers/solver-HQP-output.hpp"
#include "tsid/solvers/fwd.hpp"
#include "tsid/bindings/python/utils/container.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename Solver>
struct SolverOSQPPythonVisitor
    : public boost::python::def_visitor<SolverOSQPPythonVisitor<Solver> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl.def(bp::init<const std::string&>((bp::arg("name")),
                                        "Default Constructor with name"))
        .def("resize", &SolverOSQPPythonVisitor::resize,
             bp::args("n", "neq", "nin"))
        .add_property("ObjVal", &Solver::getObjectiveValue, "return obj value")
        .def("solve", &SolverOSQPPythonVisitor::solve, bp::args("HQPData"))
        .def("solve", &SolverOSQPPythonVisitor::solver_helper,
             bp::args("HQPData for Python"))
        .add_property("qpData", &Solver::getQPData, "return QP Data object")
        .def("retrieveQPData", &SolverOSQPPythonVisitor::retrieveQPData,
             bp::args("HQPData for Python"))
        .def("set_maximum_iterations", &Solver::setMaximumIterations)
        .def("set_sigma", &Solver::setSigma)
        .def("set_alpha", &Solver::setAlpha)
        .def("set_rho", &Solver::setRho)
        .def("set_epsilon_absolute", &Solver::setEpsilonAbsolute)
        .def("set_epsilon_relative", &Solver::setEpsilonRelative)
        .def("set_verbose", &Solver::setVerbose);
  }

  static void resize(Solver& self, unsigned int n, unsigned int neq,
                     unsigned int nin) {
    return self.resize(n, neq, nin);
  }
  static solvers::HQPOutput solve(Solver& self,
                                  const solvers::HQPData& problemData) {
    solvers::HQPOutput output;
    output = self.solve(problemData);
    return output;
  }
  static solvers::HQPOutput solver_helper(Solver& self, HQPDatas& HQPDatas) {
    solvers::HQPOutput output;
    solvers::HQPData& data = HQPDatas.get();

    output = self.solve(data);

    return output;
  }

  static solvers::QPData retrieveQPData(Solver& self, HQPDatas& HQPDatas) {
    solvers::HQPData& data = HQPDatas.get();
    self.retrieveQPData(data);
    return self.getQPData();
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Solver osqp info.";
    bp::class_<Solver>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(SolverOSQPPythonVisitor<Solver>());
  }
};

}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_solver_osqp_hpp__

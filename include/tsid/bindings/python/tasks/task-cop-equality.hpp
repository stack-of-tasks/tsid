//
// Copyright (c) 2021 University of Trento
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

#ifndef __tsid_python_task_cop_hpp__
#define __tsid_python_task_cop_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-cop-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename TaskCOP>
struct TaskCOPEqualityPythonVisitor
    : public boost::python::def_visitor<
          TaskCOPEqualityPythonVisitor<TaskCOP> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl.def(bp::init<std::string, robots::RobotWrapper&>(
               (bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &TaskCOP::dim, "return dimension size")
        .def("setReference", &TaskCOPEqualityPythonVisitor::setReference,
             bp::arg("ref"))
        .def("setContactNormal",
             &TaskCOPEqualityPythonVisitor::setContactNormal, bp::arg("normal"))
        .def("compute", &TaskCOPEqualityPythonVisitor::compute,
             bp::args("t", "q", "v", "data"))
        .def("getConstraint", &TaskCOPEqualityPythonVisitor::getConstraint)
        .add_property("name", &TaskCOPEqualityPythonVisitor::name);
  }
  static std::string name(TaskCOP& self) {
    std::string name = self.name();
    return name;
  }
  static math::ConstraintEquality compute(TaskCOP& self, const double t,
                                          const Eigen::VectorXd& q,
                                          const Eigen::VectorXd& v,
                                          pinocchio::Data& data) {
    self.compute(t, q, v, data);
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }
  static math::ConstraintEquality getConstraint(const TaskCOP& self) {
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }
  static void setReference(TaskCOP& self, const Eigen::Vector3d& ref) {
    self.setReference(ref);
  }
  static void setContactNormal(TaskCOP& self, const Eigen::Vector3d& n) {
    self.setContactNormal(n);
  }
  static void expose(const std::string& class_name) {
    std::string doc = "TaskCOPEqualityPythonVisitor info.";
    bp::class_<TaskCOP>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(TaskCOPEqualityPythonVisitor<TaskCOP>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_task_cop_hpp__

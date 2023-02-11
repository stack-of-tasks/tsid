//
// Copyright (c) 2018 CNRS
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

#ifndef __tsid_python_task_actuation_bounds_hpp__
#define __tsid_python_task_actuation_bounds_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-actuation-bounds.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-base.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename Task>
struct TaskActuationBoundsPythonVisitor
    : public boost::python::def_visitor<
          TaskActuationBoundsPythonVisitor<Task> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl.def(bp::init<std::string, robots::RobotWrapper&>(
               (bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &Task::dim, "return dimension size")
        .add_property("mask",
                      bp::make_function(
                          &TaskActuationBoundsPythonVisitor::getmask,
                          bp::return_value_policy<bp::copy_const_reference>()),
                      "Return mask")
        .def("setMask", &TaskActuationBoundsPythonVisitor::setmask,
             bp::arg("mask"))
        .def("setBounds", &TaskActuationBoundsPythonVisitor::setBounds,
             bp::args("lower", "upper"))
        .def("compute", &TaskActuationBoundsPythonVisitor::compute,
             bp::args("t", "q", "v", "data"))
        .def("getConstraint", &TaskActuationBoundsPythonVisitor::getConstraint)
        .add_property("getLowerBounds",
                      bp::make_function(
                          &TaskActuationBoundsPythonVisitor::getLowerBounds,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("getUpperBounds",
                      bp::make_function(
                          &TaskActuationBoundsPythonVisitor::getUpperBounds,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("name", &TaskActuationBoundsPythonVisitor::name);
  }
  static std::string name(Task& self) {
    std::string name = self.name();
    return name;
  }
  static math::ConstraintInequality compute(Task& self, const double t,
                                            const Eigen::VectorXd& q,
                                            const Eigen::VectorXd& v,
                                            pinocchio::Data& data) {
    self.compute(t, q, v, data);
    math::ConstraintInequality cons(
        self.getConstraint().name(), self.getConstraint().matrix(),
        self.getConstraint().lowerBound(), self.getConstraint().upperBound());
    return cons;
  }
  static math::ConstraintInequality getConstraint(const Task& self) {
    math::ConstraintInequality cons(
        self.getConstraint().name(), self.getConstraint().matrix(),
        self.getConstraint().lowerBound(), self.getConstraint().upperBound());
    return cons;
  }
  static const Eigen::VectorXd& getmask(const Task& self) {
    return self.mask();
  }
  static void setmask(Task& self, const Eigen::VectorXd mask) {
    return self.mask(mask);
  }
  static const Eigen::VectorXd& getLowerBounds(const Task& self) {
    return self.getLowerBounds();
  }
  static const Eigen::VectorXd& getUpperBounds(const Task& self) {
    return self.getUpperBounds();
  }
  static void setBounds(Task& self, const Eigen::VectorXd lower,
                        const Eigen::VectorXd upper) {
    return self.setBounds(lower, upper);
  }
  static void expose(const std::string& class_name) {
    std::string doc = "Task info.";
    bp::class_<Task>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(TaskActuationBoundsPythonVisitor<Task>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_task_actuation_bounds_hpp__

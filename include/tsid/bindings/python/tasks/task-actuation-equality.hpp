/*
 * Author:  Ram Charan Teja Thota
 * Date:    August 1, 2024
 *
 * Description:
 * this file contain boost python binding for task acutation equality
 */
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

#ifndef __tsid_python_task_actuation_equality_hpp__
#define __tsid_python_task_actuation_equality_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/math/constraint-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/tasks/task-actuation-equality.hpp"
#include "tsid/trajectories/trajectory-base.hpp"

namespace tsid {

namespace python {
namespace bp = boost::python;

template <typename TaskAucEq>
struct TaskActuationEqualityPythonVisitor
    : public boost::python::def_visitor<
          TaskActuationEqualityPythonVisitor<TaskAucEq>> {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl.def(bp::init<std::string, robots::RobotWrapper&>(
               (bp::arg("name"), bp::arg("robot")), "Default Constructor"))

        .add_property("dim", &TaskAucEq::dim, "return dimension size")

        .def("compute", &TaskActuationEqualityPythonVisitor::compute,
             bp::args("t", "q", "v", "data"))

        .def("getConstraint",
             &TaskActuationEqualityPythonVisitor::getConstraint)

        .add_property("mask",
                      bp::make_function(
                          &TaskActuationEqualityPythonVisitor::getmask,
                          bp::return_value_policy<bp::copy_const_reference>()),
                      "Return mask")

        .def("setMask", &TaskActuationEqualityPythonVisitor::setmask,
             bp::arg("mask"))

        .def("setReference", &TaskActuationEqualityPythonVisitor::setReference,
             bp::arg("ref"))
        .def("getReference", &TaskActuationEqualityPythonVisitor::getReference,
             bp::return_value_policy<bp::copy_const_reference>())

        .def("setWeightVector",
             &TaskActuationEqualityPythonVisitor::setWeightVector,
             bp::arg("weights"))
        .def("getWeightVector",
             &TaskActuationEqualityPythonVisitor::getWeightVector,
             bp::return_value_policy<bp::copy_const_reference>());
  }

  static std::string name(TaskAucEq& self) {
    std::string name = self.name();
    return name;
  }

  static math::ConstraintEquality compute(TaskAucEq& self, const double t,
                                          const Eigen::VectorXd& q,
                                          const Eigen::VectorXd& v,
                                          pinocchio::Data& data) {
    self.compute(t, q, v, data);
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }

  static math::ConstraintEquality getConstraint(const TaskAucEq& self) {
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }

  //// getter and setter of reference
  static void setReference(TaskAucEq& self, const Eigen::VectorXd& ref) {
    self.setReference(ref);
  }

  static const Eigen::VectorXd& getReference(const TaskAucEq& self) {
    return self.getReference();
  }

  // getter and setter of weight
  static void setWeightVector(TaskAucEq& self, const Eigen::VectorXd& weights) {
    self.setWeightVector(weights);
  }

  static const Eigen::VectorXd& getWeightVector(const TaskAucEq& self) {
    return self.getWeightVector();
  }

  // getter and setter of mask
  static const Eigen::VectorXd& getmask(const TaskAucEq& self) {
    return self.mask();
  }

  static void setmask(TaskAucEq& self, const Eigen::VectorXd mask) {
    return self.mask(mask);
  }

  static void expose(const std::string& class_name) {
    std::string doc = "TaskActuationEqualityPythonVisitor info.";
    bp::class_<TaskAucEq>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(TaskActuationEqualityPythonVisitor<TaskAucEq>());

    bp::register_ptr_to_python<boost::shared_ptr<math::ConstraintBase>>();
  }
};

}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_task_actuation_equality_hpp__
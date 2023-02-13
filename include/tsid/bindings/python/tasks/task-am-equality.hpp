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

#ifndef __tsid_python_task_am_hpp__
#define __tsid_python_task_am_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-angular-momentum-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename TaskAM>
struct TaskAMEqualityPythonVisitor
    : public boost::python::def_visitor<TaskAMEqualityPythonVisitor<TaskAM> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl.def(bp::init<std::string, robots::RobotWrapper&>(
               (bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &TaskAM::dim, "return dimension size")
        .def("setReference", &TaskAMEqualityPythonVisitor::setReference,
             bp::arg("ref"))
        .add_property(
            "getDesiredMomentumDerivative",
            bp::make_function(
                &TaskAMEqualityPythonVisitor::getDesiredMomentumDerivative,
                bp::return_value_policy<bp::copy_const_reference>()),
            "Return dL_desired")
        .def("getdMomentum", &TaskAMEqualityPythonVisitor::getdMomentum,
             bp::arg("dv"))
        .add_property("momentum_error",
                      bp::make_function(
                          &TaskAMEqualityPythonVisitor::momentum_error,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("momentum",
                      bp::make_function(
                          &TaskAMEqualityPythonVisitor::momentum,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("momentum_ref",
                      bp::make_function(
                          &TaskAMEqualityPythonVisitor::momentum_ref,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("dmomentum_ref",
                      bp::make_function(
                          &TaskAMEqualityPythonVisitor::dmomentum_ref,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp",
                      bp::make_function(
                          &TaskAMEqualityPythonVisitor::Kp,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd",
                      bp::make_function(
                          &TaskAMEqualityPythonVisitor::Kd,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskAMEqualityPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskAMEqualityPythonVisitor::setKd, bp::arg("Kd"))
        .def("compute", &TaskAMEqualityPythonVisitor::compute,
             bp::args("t", "q", "v", "data"))
        .def("getConstraint", &TaskAMEqualityPythonVisitor::getConstraint)
        .add_property("name", &TaskAMEqualityPythonVisitor::name);
  }
  static std::string name(TaskAM& self) {
    std::string name = self.name();
    return name;
  }
  static math::ConstraintEquality compute(TaskAM& self, const double t,
                                          const Eigen::VectorXd& q,
                                          const Eigen::VectorXd& v,
                                          pinocchio::Data& data) {
    self.compute(t, q, v, data);
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }
  static math::ConstraintEquality getConstraint(const TaskAM& self) {
    math::ConstraintEquality cons(self.getConstraint().name(),
                                  self.getConstraint().matrix(),
                                  self.getConstraint().vector());
    return cons;
  }
  static void setReference(TaskAM& self,
                           const trajectories::TrajectorySample& ref) {
    self.setReference(ref);
  }
  static const Eigen::Vector3d& getDesiredMomentumDerivative(
      const TaskAM& self) {
    return self.getDesiredMomentumDerivative();
  }
  static Eigen::Vector3d getdMomentum(TaskAM& self, const Eigen::VectorXd dv) {
    return self.getdMomentum(dv);
  }
  static const Eigen::Vector3d& momentum_error(const TaskAM& self) {
    return self.momentum_error();
  }
  static const Eigen::Vector3d& momentum(const TaskAM& self) {
    return self.momentum();
  }
  static const Eigen::VectorXd& momentum_ref(const TaskAM& self) {
    return self.momentum_ref();
  }
  static const Eigen::VectorXd& dmomentum_ref(const TaskAM& self) {
    return self.dmomentum_ref();
  }
  static const Eigen::Vector3d& Kp(TaskAM& self) { return self.Kp(); }
  static const Eigen::Vector3d& Kd(TaskAM& self) { return self.Kd(); }
  static void setKp(TaskAM& self, const ::Eigen::VectorXd Kp) {
    return self.Kp(Kp);
  }
  static void setKd(TaskAM& self, const ::Eigen::VectorXd Kv) {
    return self.Kd(Kv);
  }
  static void expose(const std::string& class_name) {
    std::string doc = "TaskAMEqualityPythonVisitor info.";
    bp::class_<TaskAM>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(TaskAMEqualityPythonVisitor<TaskAM>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_task_am_hpp__

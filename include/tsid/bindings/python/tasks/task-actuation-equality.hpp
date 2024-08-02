/*
 * Author:  Ram Charan Teja Thota
 * Date:    August 1, 2024
 *
 * Description:
 * this file contain boost python binding for task acutation equality
 */

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

  void visit(PyClass &cl) const {
    cl.def(bp::init<std::string, robots::RobotWrapper &>(
            (bp::arg("name"),bp::arg("robot")),"Default Constructor"))

        .add_property("dim", &TaskAucEq::dim, "return dimension size")

        .add_property("mask",
                      bp::make_function(
                          &TaskActuationEqualityPythonVisitor::getmask,
                          bp::return_value_policy<bp::copy_const_reference>()),
                      "Return mask")
        .add_property("getReference",
                     bp::make_function(
                         &TaskCOMEqualityPythonVisitor::getReference,
                         bp::return_value_policy<bp::copy_const_reference>()))

        .def("setMask", &TaskActuationEqualityPythonVisitor::setmask,
             bp::arg("mask"))

        .def("compute", &TaskActuationEqualityPythonVisitor::compute,
             bp::args("t", "q", "v", "data"))

        .def("getConstraint", &TaskActuationEqualityPythonVisitor::getConstraint)

        .def("setReference", &TaskActuationEqualityPythonVisitor::setReference,
             bp::arg("ref"))

        .def("setWeightVector", &TaskActuationEqualityPythonVisitor::setWeightVector,
             bp::arg("weights"))

        .def("getReference", &TaskActuationEqualityPythonVisitor::getReference)
         .def("setReference", &TaskActuationEqualityPythonVisitor::setReference)
        

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

 static void setReference(TaskAucEq& self,
                          const Eigen::VectorXd& ref) {
   self.setReference(ref);
 }

 static const Eigen::VectorXd& getReference(const TaskAucEq& self) {
   return self.getReference();
 }

 static void setWeightVector(TaskAucEq& self,
                          const Eigen::VectorXd& ref) {
   self.setReference(ref);
 }

 static const Eigen::VectorXd& getWeightVector(const TaskAucEq& self) {
   return self.getWeightVector();
 }






};

} // namespace python
} // namespace tsid

#endif // ifndef __tsid_python_task_actuation_equality_hpp__
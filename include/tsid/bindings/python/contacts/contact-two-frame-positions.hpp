//
// Copyright (c) 2023 MIPT
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

#ifndef __tsid_python_contact_two_frame_positions_hpp__
#define __tsid_python_contact_two_frame_positions_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/contacts/contact-two-frame-positions.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename ContactTwoFramePositions>
struct ContactTwoFramePositionsPythonVisitor
    : public boost::python::def_visitor<
          ContactTwoFramePositionsPythonVisitor<ContactTwoFramePositions> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl
        .def(bp::init<std::string, robots::RobotWrapper&, std::string,
                      std::string, double, double>(
            (bp::arg("name"), bp::arg("robot"), bp::arg("framename1"),
             bp::arg("framename2"), bp::arg("minForce"), bp::arg("maxForce")),
            "Default Constructor"))
        .add_property("n_motion", &ContactTwoFramePositions::n_motion,
                      "return number of motion")
        .add_property("n_force", &ContactTwoFramePositions::n_force,
                      "return number of force")
        .add_property("name", &ContactTwoFramePositionsPythonVisitor::name,
                      "return name")
        .def("computeMotionTask",
             &ContactTwoFramePositionsPythonVisitor::computeMotionTask,
             bp::args("t", "q", "v", "data"))
        .def("computeForceTask",
             &ContactTwoFramePositionsPythonVisitor::computeForceTask,
             bp::args("t", "q", "v", "data"))
        .def("computeForceRegularizationTask",
             &ContactTwoFramePositionsPythonVisitor::
                 computeForceRegularizationTask,
             bp::args("t", "q", "v", "data"))

        .add_property(
            "getForceGeneratorMatrix",
            bp::make_function(
                &ContactTwoFramePositionsPythonVisitor::getForceGeneratorMatrix,
                bp::return_value_policy<bp::copy_const_reference>()))

        .def("getNormalForce",
             &ContactTwoFramePositionsPythonVisitor::getNormalForce,
             bp::arg("vec"))
        .add_property("getMinNormalForce",
                      &ContactTwoFramePositions::getMinNormalForce)
        .add_property("getMaxNormalForce",
                      &ContactTwoFramePositions::getMaxNormalForce)

        .add_property("Kp",
                      bp::make_function(
                          &ContactTwoFramePositionsPythonVisitor::Kp,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd",
                      bp::make_function(
                          &ContactTwoFramePositionsPythonVisitor::Kd,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &ContactTwoFramePositionsPythonVisitor::setKp,
             bp::arg("Kp"))
        .def("setKd", &ContactTwoFramePositionsPythonVisitor::setKd,
             bp::arg("Kd"))
        .def("setContactNormal",
             &ContactTwoFramePositionsPythonVisitor::setContactNormal,
             bp::args("vec"))
        .def("setFrictionCoefficient",
             &ContactTwoFramePositionsPythonVisitor::setFrictionCoefficient,
             bp::args("friction_coeff"))
        .def("setMinNormalForce",
             &ContactTwoFramePositionsPythonVisitor::setMinNormalForce,
             bp::args("min_force"))
        .def("setMaxNormalForce",
             &ContactTwoFramePositionsPythonVisitor::setMaxNormalForce,
             bp::args("max_force"))
        .def("setForceReference",
             &ContactTwoFramePositionsPythonVisitor::setForceReference,
             bp::args("f_vec"))
        .def("setRegularizationTaskWeightVector",
             &ContactTwoFramePositionsPythonVisitor::
                 setRegularizationTaskWeightVector,
             bp::args("w_vec"));
  }
  static std::string name(ContactTwoFramePositions& self) {
    std::string name = self.name();
    return name;
  }

  static math::ConstraintEquality computeMotionTask(
      ContactTwoFramePositions& self, const double t, const Eigen::VectorXd& q,
      const Eigen::VectorXd& v, pinocchio::Data& data) {
    self.computeMotionTask(t, q, v, data);
    math::ConstraintEquality cons(self.getMotionConstraint().name(),
                                  self.getMotionConstraint().matrix(),
                                  self.getMotionConstraint().vector());
    return cons;
  }
  static math::ConstraintInequality computeForceTask(
      ContactTwoFramePositions& self, const double t, const Eigen::VectorXd& q,
      const Eigen::VectorXd& v, const pinocchio::Data& data) {
    self.computeForceTask(t, q, v, data);
    math::ConstraintInequality cons(self.getForceConstraint().name(),
                                    self.getForceConstraint().matrix(),
                                    self.getForceConstraint().lowerBound(),
                                    self.getForceConstraint().upperBound());
    return cons;
  }
  static math::ConstraintEquality computeForceRegularizationTask(
      ContactTwoFramePositions& self, const double t, const Eigen::VectorXd& q,
      const Eigen::VectorXd& v, const pinocchio::Data& data) {
    self.computeForceRegularizationTask(t, q, v, data);
    math::ConstraintEquality cons(self.getForceRegularizationTask().name(),
                                  self.getForceRegularizationTask().matrix(),
                                  self.getForceRegularizationTask().vector());
    return cons;
  }

  static const Eigen::MatrixXd& getForceGeneratorMatrix(
      ContactTwoFramePositions& self) {
    return self.getForceGeneratorMatrix();
  }
  static const Eigen::VectorXd& Kp(ContactTwoFramePositions& self) {
    return self.Kp();
  }
  static const Eigen::VectorXd& Kd(ContactTwoFramePositions& self) {
    return self.Kd();
  }
  static void setKp(ContactTwoFramePositions& self,
                    const ::Eigen::VectorXd Kp) {
    return self.Kp(Kp);
  }
  static void setKd(ContactTwoFramePositions& self,
                    const ::Eigen::VectorXd Kd) {
    return self.Kd(Kd);
  }
  static bool setContactTwoFramePositionss(
      ContactTwoFramePositions& self,
      const ::Eigen::MatrixXd ContactTwoFramePositionss) {
    return self.setContactTwoFramePositionss(ContactTwoFramePositionss);
  }
  static bool setContactNormal(ContactTwoFramePositions& self,
                               const ::Eigen::VectorXd contactNormal) {
    return self.setContactNormal(contactNormal);
  }
  static bool setFrictionCoefficient(ContactTwoFramePositions& self,
                                     const double frictionCoefficient) {
    return self.setFrictionCoefficient(frictionCoefficient);
  }
  static bool setMinNormalForce(ContactTwoFramePositions& self,
                                const double minNormalForce) {
    return self.setMinNormalForce(minNormalForce);
  }
  static bool setMaxNormalForce(ContactTwoFramePositions& self,
                                const double maxNormalForce) {
    return self.setMaxNormalForce(maxNormalForce);
  }
  static void setForceReference(ContactTwoFramePositions& self,
                                const ::Eigen::VectorXd f_ref) {
    self.setForceReference(f_ref);
  }
  static void setRegularizationTaskWeightVector(ContactTwoFramePositions& self,
                                                const ::Eigen::VectorXd w) {
    self.setRegularizationTaskWeightVector(w);
  }
  static double getNormalForce(ContactTwoFramePositions& self,
                               Eigen::VectorXd f) {
    return self.getNormalForce(f);
  }
  static void expose(const std::string& class_name) {
    std::string doc = "ContactTwoFramePositions info.";
    bp::class_<ContactTwoFramePositions>(class_name.c_str(), doc.c_str(),
                                         bp::no_init)
        .def(ContactTwoFramePositionsPythonVisitor<ContactTwoFramePositions>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_contact_two_frame_positions_hpp__

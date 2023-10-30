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

#ifndef __tsid_python_contact_two_frames_hpp__
#define __tsid_python_contact_two_frames_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/contacts/contact-two-frames.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename ContactTwoFrames>
struct ContactTwoFramesPythonVisitor
    : public boost::python::def_visitor<
          ContactTwoFramesPythonVisitor<ContactTwoFrames> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl
        .def(bp::init<std::string, robots::RobotWrapper&, std::string,
                      std::string, double, double>(
            (bp::arg("name"), bp::arg("robot"), bp::arg("framename1"),
             bp::arg("framename2"), bp::arg("minForce"), bp::arg("maxForce")),
            "Default Constructor"))
        .add_property("n_motion", &ContactTwoFrames::n_motion,
                      "return number of motion")
        .add_property("n_force", &ContactTwoFrames::n_force,
                      "return number of force")
        .add_property("name", &ContactTwoFramesPythonVisitor::name,
                      "return name")
        .def("computeMotionTask",
             &ContactTwoFramesPythonVisitor::computeMotionTask,
             bp::args("t", "q", "v", "data"))
        .def("computeForceTask",
             &ContactTwoFramesPythonVisitor::computeForceTask,
             bp::args("t", "q", "v", "data"))
        .def("computeForceRegularizationTask",
             &ContactTwoFramesPythonVisitor::computeForceRegularizationTask,
             bp::args("t", "q", "v", "data"))

        .add_property(
            "getForceGeneratorMatrix",
            bp::make_function(
                &ContactTwoFramesPythonVisitor::getForceGeneratorMatrix,
                bp::return_value_policy<bp::copy_const_reference>()))

        .def("getNormalForce", &ContactTwoFramesPythonVisitor::getNormalForce,
             bp::arg("vec"))
        .add_property("getMinNormalForce", &ContactTwoFrames::getMinNormalForce)
        .add_property("getMaxNormalForce", &ContactTwoFrames::getMaxNormalForce)

        .add_property("Kp",
                      bp::make_function(
                          &ContactTwoFramesPythonVisitor::Kp,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd",
                      bp::make_function(
                          &ContactTwoFramesPythonVisitor::Kd,
                          bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &ContactTwoFramesPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &ContactTwoFramesPythonVisitor::setKd, bp::arg("Kd"))

        .def("useLocalFrame", &ContactTwoFramesPythonVisitor::useLocalFrame,
             bp::arg("local_frame"))

        .def("setContactNormal",
             &ContactTwoFramesPythonVisitor::setContactNormal, bp::args("vec"))
        .def("setFrictionCoefficient",
             &ContactTwoFramesPythonVisitor::setFrictionCoefficient,
             bp::args("friction_coeff"))
        .def("setMinNormalForce",
             &ContactTwoFramesPythonVisitor::setMinNormalForce,
             bp::args("min_force"))
        .def("setMaxNormalForce",
             &ContactTwoFramesPythonVisitor::setMaxNormalForce,
             bp::args("max_force"))
        .def("setReference", &ContactTwoFramesPythonVisitor::setReference,
             bp::args("SE3"))
        .def("setForceReference",
             &ContactTwoFramesPythonVisitor::setForceReference,
             bp::args("f_vec"))
        .def("setRegularizationTaskWeightVector",
             &ContactTwoFramesPythonVisitor::setRegularizationTaskWeightVector,
             bp::args("w_vec"));
  }
  static std::string name(ContactTwoFrames& self) {
    std::string name = self.name();
    return name;
  }

  static math::ConstraintEquality computeMotionTask(ContactTwoFrames& self,
                                                    const double t,
                                                    const Eigen::VectorXd& q,
                                                    const Eigen::VectorXd& v,
                                                    pinocchio::Data& data) {
    self.computeMotionTask(t, q, v, data);
    math::ConstraintEquality cons(self.getMotionConstraint().name(),
                                  self.getMotionConstraint().matrix(),
                                  self.getMotionConstraint().vector());
    return cons;
  }
  static math::ConstraintInequality computeForceTask(
      ContactTwoFrames& self, const double t, const Eigen::VectorXd& q,
      const Eigen::VectorXd& v, const pinocchio::Data& data) {
    self.computeForceTask(t, q, v, data);
    math::ConstraintInequality cons(self.getForceConstraint().name(),
                                    self.getForceConstraint().matrix(),
                                    self.getForceConstraint().lowerBound(),
                                    self.getForceConstraint().upperBound());
    return cons;
  }
  static math::ConstraintEquality computeForceRegularizationTask(
      ContactTwoFrames& self, const double t, const Eigen::VectorXd& q,
      const Eigen::VectorXd& v, const pinocchio::Data& data) {
    self.computeForceRegularizationTask(t, q, v, data);
    math::ConstraintEquality cons(self.getForceRegularizationTask().name(),
                                  self.getForceRegularizationTask().matrix(),
                                  self.getForceRegularizationTask().vector());
    return cons;
  }

  static void useLocalFrame(ContactTwoFrames& self, const bool local_frame) {
    self.useLocalFrame(local_frame);
  }
  static const Eigen::MatrixXd& getForceGeneratorMatrix(
      ContactTwoFrames& self) {
    return self.getForceGeneratorMatrix();
  }
  static const Eigen::VectorXd& Kp(ContactTwoFrames& self) { return self.Kp(); }
  static const Eigen::VectorXd& Kd(ContactTwoFrames& self) { return self.Kd(); }
  static void setKp(ContactTwoFrames& self, const ::Eigen::VectorXd Kp) {
    return self.Kp(Kp);
  }
  static void setKd(ContactTwoFrames& self, const ::Eigen::VectorXd Kd) {
    return self.Kd(Kd);
  }
  static bool setContactTwoFramess(ContactTwoFrames& self,
                                   const ::Eigen::MatrixXd ContactTwoFramess) {
    return self.setContactTwoFramess(ContactTwoFramess);
  }
  static bool setContactNormal(ContactTwoFrames& self,
                               const ::Eigen::VectorXd contactNormal) {
    return self.setContactNormal(contactNormal);
  }
  static bool setFrictionCoefficient(ContactTwoFrames& self,
                                     const double frictionCoefficient) {
    return self.setFrictionCoefficient(frictionCoefficient);
  }
  static bool setMinNormalForce(ContactTwoFrames& self,
                                const double minNormalForce) {
    return self.setMinNormalForce(minNormalForce);
  }
  static bool setMaxNormalForce(ContactTwoFrames& self,
                                const double maxNormalForce) {
    return self.setMaxNormalForce(maxNormalForce);
  }
  static void setReference(ContactTwoFrames& self, const pinocchio::SE3& ref) {
    self.setReference(ref);
  }
  static void setForceReference(ContactTwoFrames& self,
                                const ::Eigen::VectorXd f_ref) {
    self.setForceReference(f_ref);
  }
  static void setRegularizationTaskWeightVector(ContactTwoFrames& self,
                                                const ::Eigen::VectorXd w) {
    self.setRegularizationTaskWeightVector(w);
  }
  static double getNormalForce(ContactTwoFrames& self, Eigen::VectorXd f) {
    return self.getNormalForce(f);
  }

  static void expose(const std::string& class_name) {
    std::string doc = "ContactTwoFrames info.";
    bp::class_<ContactTwoFrames>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(ContactTwoFramesPythonVisitor<ContactTwoFrames>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_contact_two_frames_hpp__

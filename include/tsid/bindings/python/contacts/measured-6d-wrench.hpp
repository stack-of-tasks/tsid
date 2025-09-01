//
// Copyright (c) 2022 CNRS INRIA LORIA
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
// tsid If not, see <http://www.gnu.org/licenses/>.
//

#ifndef __tsid_python_measured_6d_wrench_hpp__
#define __tsid_python_measured_6d_wrench_hpp__

#include "tsid/bindings/python/fwd.hpp"
#include "tsid/contacts/measured-6d-wrench.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "pinocchio/multibody/data.hpp"

namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename Measured6dWrench>
struct Measured6dWrenchPythonVisitor
    : public boost::python::def_visitor<
          Measured6dWrenchPythonVisitor<Measured6dWrench> > {
  template <class PyClass>
  void visit(PyClass& cl) const {
    cl
        // Expose the constructor:
        .def(bp::init<std::string, typename Measured6dWrench::RobotWrapper&,
                      std::string>(
            (bp::arg("name"), bp::arg("robot"), bp::arg("frameName")),
            "Constructor for Measured6dwrench"))

        // Expose the name getter and setter:
        .add_property("name", &Measured6dWrenchPythonVisitor::getName,
                      &Measured6dWrenchPythonVisitor::setName,
                      "Get or set the name of the measured-6Dwrench instance")

        // Expose computeJointTorques(Data &data)
        .def("computeJointTorques",
             &Measured6dWrenchPythonVisitor::computeJointTorques,
             bp::args("data"),
             "Compute the joint torques from the measured contact force")

        // Expose setMeasuredContactForce(const Vector6 &fext)
        .def("setMeasuredContactForce",
             &Measured6dWrenchPythonVisitor::setMeasuredContactForce,
             bp::args("fext"), "Set the measured contact force")

        // Expose getMeasuredContactForce() as a read-only property.
        .add_property(
            "measuredContactForce",
            bp::make_function(
                &Measured6dWrenchPythonVisitor::getMeasuredContactForce),
            "Get the measured contact force")

        // Expose useLocalFrame(bool local_frame)
        .def("useLocalFrame", &Measured6dWrenchPythonVisitor::useLocalFrame,
             bp::args("local_frame"),
             "Specify whether to use the local frame for external force and "
             "jacobian");
  }

  // Wrapper for name() getter.
  static std::string getName(Measured6dWrench& self) { return self.name(); }

  // Wrapper for name(const std::string &) setter.
  static void setName(Measured6dWrench& self, const std::string& name) {
    self.name(name);
  }

  // Wrapper for computeJointTorques(Data &data) returning by value.
  static typename Measured6dWrench::Vector computeJointTorques(
      Measured6dWrench& self, pinocchio::Data& data) {
    return self.computeJointTorques(data);
  }

  // Wrapper for setMeasuredContactForce(const Vector6 &fext)
  static void setMeasuredContactForce(
      Measured6dWrench& self, const typename Measured6dWrench::Vector6& fext) {
    self.setMeasuredContactForce(fext);
  }

  // Wrapper for getMeasuredContactForce() returning by value.
  static typename Measured6dWrench::Vector6 getMeasuredContactForce(
      Measured6dWrench& self) {
    return self.getMeasuredContactForce();
  }

  // Wrapper for useLocalFrame(bool local_frame)
  static void useLocalFrame(Measured6dWrench& self, bool local_frame) {
    self.useLocalFrame(local_frame);
  }

  // Function to expose the binding.
  static void expose(const std::string& class_name) {
    std::string doc = "Bindings for tsid::contacts::Measured6dwrench";
    bp::class_<Measured6dWrench>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(Measured6dWrenchPythonVisitor<Measured6dWrench>());
  }
};

}  // namespace python
}  // namespace tsid

#endif  // __tsid_python_measured_6d_wrench_hpp__

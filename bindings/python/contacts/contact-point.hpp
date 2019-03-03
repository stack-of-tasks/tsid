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

#ifndef __tsid_python_contact_6d_hpp__
#define __tsid_python_contact_6d_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/contacts/contact-point.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename ContactPoint>
    struct ContactPointPythonVisitor
    : public boost::python::def_visitor< ContactPointPythonVisitor<ContactPoint> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, std::string, Eigen::VectorXd, double, double, double> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename"), bp::arg("contactNormal"), bp::arg("frictionCoeff"), bp::arg("minForce"), bp::arg("maxForce")), "Default Constructor"))
        .add_property("n_motion", &ContactPoint::n_motion, "return number of motion")
        .add_property("n_force", &ContactPoint::n_force, "return number of force")
        .add_property("name", &ContactPointPythonVisitor::name, "return name")
        .def("computeMotionTask", &ContactPointPythonVisitor::computeMotionTask, bp::args("t", "q", "v", "data"))
        .def("computeForceTask", &ContactPointPythonVisitor::computeForceTask, bp::args("t", "q", "v", "data"))
        .def("computeForceRegularizationTask", &ContactPointPythonVisitor::computeForceRegularizationTask, bp::args("t", "q", "v", "data"))
        
        .add_property("getForceGeneratorMatrix", bp::make_function(&ContactPointPythonVisitor::getForceGeneratorMatrix, bp::return_value_policy<bp::copy_const_reference>()))
        
        .def("getNormalForce", &ContactPointPythonVisitor::getNormalForce, bp::arg("vec"))
        .add_property("getMinNormalForce", &ContactPoint::getMinNormalForce)
        .add_property("getMaxNormalForce", &ContactPoint::getMaxNormalForce)
        
        .add_property("Kp", bp::make_function(&ContactPointPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&ContactPointPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &ContactPointPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &ContactPointPythonVisitor::setKd, bp::arg("Kd"))

        .def("useLocalFrame", &ContactPointPythonVisitor::useLocalFrame, bp::arg("local_frame"))

        .def("setContactNormal", &ContactPointPythonVisitor::setContactNormal, bp::args("vec"))
        .def("setFrictionCoefficient", &ContactPointPythonVisitor::setFrictionCoefficient, bp::args("friction_coeff"))
        .def("setMinNormalForce", &ContactPointPythonVisitor::setMinNormalForce, bp::args("min_force"))
        .def("setMaxNormalForce", &ContactPointPythonVisitor::setMaxNormalForce, bp::args("max_force"))
        .def("setReference", &ContactPointPythonVisitor::setReference, bp::args("SE3"))
        .def("setForceReference", &ContactPointPythonVisitor::setForceReference, bp::args("f_vec"))
        .def("setRegularizationTaskWeightVector", &ContactPointPythonVisitor::setRegularizationTaskWeightVector, bp::args("w_vec"))
        ;
      }
      static std::string name(ContactPoint & self){
        std::string name = self.name();
        return name;
      }

      static math::ConstraintEquality computeMotionTask(ContactPoint & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.computeMotionTask(t, q, v, data);
        math::ConstraintEquality cons(self.getMotionConstraint().name(), self.getMotionConstraint().matrix(), self.getMotionConstraint().vector());
        return cons;
      }
      static math::ConstraintInequality computeForceTask(ContactPoint & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.computeForceTask(t, q, v, data);
        math::ConstraintInequality cons(self.getForceConstraint().name(), self.getForceConstraint().matrix(), self.getForceConstraint().lowerBound(), self.getForceConstraint().upperBound());
        return cons;
      }
      static math::ConstraintEquality computeForceRegularizationTask(ContactPoint & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.computeForceRegularizationTask(t, q, v, data);
        math::ConstraintEquality cons(self.getForceRegularizationTask().name(), self.getForceRegularizationTask().matrix(), self.getForceRegularizationTask().vector());
        return cons;
      }

      static void useLocalFrame (ContactPoint & self, const bool local_frame) {
        self.useLocalFrame(local_frame);
      }
      static const Eigen::MatrixXd & getForceGeneratorMatrix(ContactPoint & self){
        return self.getForceGeneratorMatrix();
      }
      static const Eigen::VectorXd & Kp (ContactPoint & self){
        return self.Kp();
      }  
      static const Eigen::VectorXd & Kd (ContactPoint & self){
        return self.Kd();
      }    
      static void setKp (ContactPoint & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (ContactPoint & self, const::Eigen::VectorXd Kd){
        return self.Kd(Kd);
      }
      static bool setContactPoints (ContactPoint & self, const::Eigen::MatrixXd contactpoints){
        return self.setContactPoints(contactpoints);
      }
      static bool setContactNormal (ContactPoint & self, const::Eigen::VectorXd contactNormal){
        return self.setContactNormal(contactNormal);
      }
      static bool setFrictionCoefficient (ContactPoint & self, const double frictionCoefficient){
        return self.setFrictionCoefficient(frictionCoefficient);
      }
      static bool setMinNormalForce (ContactPoint & self, const double minNormalForce){
        return self.setMinNormalForce(minNormalForce);
      }
      static bool setMaxNormalForce (ContactPoint & self, const double maxNormalForce){
        return self.setMaxNormalForce(maxNormalForce);
      }
      static void setReference(ContactPoint & self, const pinocchio::SE3 & ref){
        self.setReference(ref);
      }
      static void setForceReference(ContactPoint & self, const::Eigen::VectorXd f_ref){
        self.setForceReference(f_ref);
      }  
      static void setRegularizationTaskWeightVector(ContactPoint & self, const::Eigen::VectorXd w){
        self.setRegularizationTaskWeightVector(w);
      }
      static double getNormalForce(ContactPoint & self, Eigen::VectorXd f){
        return self.getNormalForce(f);
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "ContactPoint info.";
        bp::class_<ContactPoint>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(ContactPointPythonVisitor<ContactPoint>());
      }
    };
  }
}


#endif // ifndef __tsid_python_contact_hpp__

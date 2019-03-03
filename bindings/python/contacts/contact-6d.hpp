//
// Copyright (c) 2018 CNRS, NYU, MPI Tübingen
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

#include "tsid/contacts/contact-6d.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
#include "tsid/tasks/task-se3-equality.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Contact6d>
    struct Contact6DPythonVisitor
    : public boost::python::def_visitor< Contact6DPythonVisitor<Contact6d> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, std::string, Eigen::MatrixXd, Eigen::VectorXd, double, double, double> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename"), bp::arg("contactPoint"), bp::arg("contactNormal"), bp::arg("frictionCoeff"), bp::arg("minForce"), bp::arg("maxForce")), "Default Constructor"))
        .def(bp::init<std::string, robots::RobotWrapper &, std::string, Eigen::MatrixXd, Eigen::VectorXd, double, double, double, double> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename"), bp::arg("contactPoint"), bp::arg("contactNormal"), bp::arg("frictionCoeff"), bp::arg("minForce"), bp::arg("maxForce"), bp::arg("wForceReg")), "Deprecated Constructor"))
        .add_property("n_motion", &Contact6d::n_motion, "return number of motion")
        .add_property("n_force", &Contact6d::n_force, "return number of force")
        .add_property("name", &Contact6DPythonVisitor::name, "return name")
        .def("computeMotionTask", &Contact6DPythonVisitor::computeMotionTask, bp::args("t", "q", "v", "data"))
        .def("computeForceTask", &Contact6DPythonVisitor::computeForceTask, bp::args("t", "q", "v", "data"))
        .def("computeForceRegularizationTask", &Contact6DPythonVisitor::computeForceRegularizationTask, bp::args("t", "q", "v", "data"))
        
        .add_property("getForceGeneratorMatrix", bp::make_function(&Contact6DPythonVisitor::getForceGeneratorMatrix, bp::return_value_policy<bp::copy_const_reference>()))
        
        .def("getNormalForce", &Contact6DPythonVisitor::getNormalForce, bp::arg("vec"))
        .add_property("getMinNormalForce", &Contact6d::getMinNormalForce)
        .add_property("getMaxNormalForce", &Contact6d::getMaxNormalForce)
        
        .add_property("Kp", bp::make_function(&Contact6DPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&Contact6DPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &Contact6DPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &Contact6DPythonVisitor::setKd, bp::arg("Kd"))
        
        .def("setContactPoints", &Contact6DPythonVisitor::setContactPoints, bp::args("vec"))
        .def("setContactNormal", &Contact6DPythonVisitor::setContactNormal, bp::args("vec"))
        .def("setFrictionCoefficient", &Contact6DPythonVisitor::setFrictionCoefficient, bp::args("friction_coeff"))
        .def("setMinNormalForce", &Contact6DPythonVisitor::setMinNormalForce, bp::args("min_force"))
        .def("setMaxNormalForce", &Contact6DPythonVisitor::setMaxNormalForce, bp::args("max_force"))
        .def("setReference", &Contact6DPythonVisitor::setReference, bp::args("SE3"))
        .def("setForceReference", &Contact6DPythonVisitor::setForceReference, bp::args("f_vec"))
        .def("setRegularizationTaskWeightVector", &Contact6DPythonVisitor::setRegularizationTaskWeightVector, bp::args("w_vec"))
        ;
      }
      static std::string name(Contact6d & self){
        std::string name = self.name();
        return name;
      }

      static math::ConstraintEquality computeMotionTask(Contact6d & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.computeMotionTask(t, q, v, data);
        math::ConstraintEquality cons(self.getMotionConstraint().name(), self.getMotionConstraint().matrix(), self.getMotionConstraint().vector());
        return cons;
      }
      static math::ConstraintInequality computeForceTask(Contact6d & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.computeForceTask(t, q, v, data);
        math::ConstraintInequality cons(self.getForceConstraint().name(), self.getForceConstraint().matrix(), self.getForceConstraint().lowerBound(), self.getForceConstraint().upperBound());
        return cons;
      }
      static math::ConstraintEquality computeForceRegularizationTask(Contact6d & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.computeForceRegularizationTask(t, q, v, data);
        math::ConstraintEquality cons(self.getForceRegularizationTask().name(), self.getForceRegularizationTask().matrix(), self.getForceRegularizationTask().vector());
        return cons;
      }

      static const Eigen::MatrixXd & getForceGeneratorMatrix(Contact6d & self){
        return self.getForceGeneratorMatrix();
      }
      static const Eigen::VectorXd & Kp (Contact6d & self){
        return self.Kp();
      }  
      static const Eigen::VectorXd & Kd (Contact6d & self){
        return self.Kd();
      }    
      static void setKp (Contact6d & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (Contact6d & self, const::Eigen::VectorXd Kd){
        return self.Kd(Kd);
      }
      static bool setContactPoints (Contact6d & self, const::Eigen::MatrixXd contactpoints){
        return self.setContactPoints(contactpoints);
      }
      static bool setContactNormal (Contact6d & self, const::Eigen::VectorXd contactNormal){
        return self.setContactNormal(contactNormal);
      }
      static bool setFrictionCoefficient (Contact6d & self, const double frictionCoefficient){
        return self.setFrictionCoefficient(frictionCoefficient);
      }
      static bool setMinNormalForce (Contact6d & self, const double minNormalForce){
        return self.setMinNormalForce(minNormalForce);
      }
      static bool setMaxNormalForce (Contact6d & self, const double maxNormalForce){
        return self.setMaxNormalForce(maxNormalForce);
      }
      static void setReference(Contact6d & self, const pinocchio::SE3 & ref){
        self.setReference(ref);
      }
      static void setForceReference(Contact6d & self, const::Eigen::VectorXd f_ref){
        self.setForceReference(f_ref);
      }  
      static void setRegularizationTaskWeightVector(Contact6d & self, const::Eigen::VectorXd w){
        self.setRegularizationTaskWeightVector(w);
      }
      static double getNormalForce(Contact6d & self, Eigen::VectorXd f){
        return self.getNormalForce(f);
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Contact6d info.";
        bp::class_<Contact6d>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(Contact6DPythonVisitor<Contact6d>());
      }
    };
  }
}


#endif // ifndef __tsid_python_contact_hpp__

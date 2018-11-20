#ifndef __tsid_python_contact_hpp__
#define __tsid_python_contact_hpp__

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
    struct ContactPythonVisitor
    : public boost::python::def_visitor< ContactPythonVisitor<Contact6d> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, std::string, Eigen::MatrixXd, Eigen::VectorXd, double, double, double, double> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename"), bp::arg("contactPoint"), bp::arg("contactNormal"), bp::arg("frictionCoeff"), bp::arg("minForce"), bp::arg("maxForce"), bp::arg("regWeight")), "Default Constructor"))
        .add_property("n_motion", &Contact6d::n_motion, "return number of motion")
        .add_property("n_force", &Contact6d::n_force, "return number of force")
        .add_property("name", &ContactPythonVisitor::name, "return name")
        .def("computeMotionTask", &ContactPythonVisitor::computeMotionTask, bp::args("t", "q", "v", "data"))
        .def("computeForceTask", &ContactPythonVisitor::computeForceTask, bp::args("t", "q", "v", "data"))
        .def("computeForceRegularizationTask", &ContactPythonVisitor::computeForceRegularizationTask, bp::args("t", "q", "v", "data"))
        
        .add_property("getForceGeneratorMatrix", bp::make_function(&ContactPythonVisitor::getForceGeneratorMatrix, bp::return_value_policy<bp::copy_const_reference>()))
        
        .add_property("getForceRegularizationWeight", &Contact6d::getForceRegularizationWeight, "return force reg weight")
        .def("getNormalForce", &ContactPythonVisitor::getNormalForce, bp::arg("vec"))
        .add_property("getMinNormalForce", &Contact6d::getMinNormalForce)
        .add_property("getMaxNormalForce", &Contact6d::getMaxNormalForce)
        
        .add_property("Kp", bp::make_function(&ContactPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&ContactPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &ContactPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &ContactPythonVisitor::setKd, bp::arg("Kd"))
        
        .def("setContactPoints", &ContactPythonVisitor::setContactPoints, bp::args("vec"))
        .def("setContactNormal", &ContactPythonVisitor::setContactNormal, bp::args("vec"))
        .def("setFrictionCoefficient", &ContactPythonVisitor::setFrictionCoefficient, bp::args("friction_coeff"))
        .def("setMinNormalForce", &ContactPythonVisitor::setMinNormalForce, bp::args("min_force"))
        .def("setMaxNormalForce", &ContactPythonVisitor::setMaxNormalForce, bp::args("max_force"))
        .def("setRegularizationTaskWeight", &ContactPythonVisitor::setRegularizationTaskWeight, bp::args("double"))
        .def("setReference", &ContactPythonVisitor::setReference, bp::args("SE3"))
        .def("setForceReference", &ContactPythonVisitor::setForceReference, bp::args("f_vec"))
        .def("setRegularizationTaskWeightVector", &ContactPythonVisitor::setRegularizationTaskWeightVector, bp::args("w_vec"))
        ;
      }
      static std::string name(Contact6d & self){
        std::string name = self.name();
        return name;
      }

      static math::ConstraintEquality computeMotionTask(Contact6d & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const se3::Data & data){
        self.computeMotionTask(t, q, v, data);
        math::ConstraintEquality cons(self.getMotionConstraint().name(), self.getMotionConstraint().matrix(), self.getMotionConstraint().vector());
        return cons;
      }
      static math::ConstraintInequality computeForceTask(Contact6d & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const se3::Data & data){
        self.computeForceTask(t, q, v, data);
        math::ConstraintInequality cons(self.getForceConstraint().name(), self.getForceConstraint().matrix(), self.getForceConstraint().lowerBound(), self.getForceConstraint().upperBound());
        return cons;
      }
      static math::ConstraintEquality computeForceRegularizationTask(Contact6d & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const se3::Data & data){
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
      static bool setRegularizationTaskWeight (Contact6d & self, const double w){
        return self.setRegularizationTaskWeight(w);
      }
      static void setReference(Contact6d & self, const se3::SE3 & ref){
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
        .def(ContactPythonVisitor<Contact6d>());
      }
    };
  }
}


#endif // ifndef __tsid_python_contact_hpp__
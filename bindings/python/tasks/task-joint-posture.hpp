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

#ifndef __tsid_python_task_joint_hpp__
#define __tsid_python_task_joint_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/tasks/task-joint-posture.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename TaskJoint>
    struct TaskJointPosturePythonVisitor
    : public boost::python::def_visitor< TaskJointPosturePythonVisitor<TaskJoint> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &> ((bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &TaskJoint::dim, "return dimension size")
        .def("setReference", &TaskJointPosturePythonVisitor::setReference, bp::arg("ref"))
        .add_property("getDesiredAcceleration", bp::make_function(&TaskJointPosturePythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .add_property("mask", bp::make_function(&TaskJointPosturePythonVisitor::getmask, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .def("mask", &TaskJointPosturePythonVisitor::setmask, bp::arg("mask"))
        .def("getAcceleration", &TaskJointPosturePythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskJointPosturePythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskJointPosturePythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position", bp::make_function(&TaskJointPosturePythonVisitor::position, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity", bp::make_function(&TaskJointPosturePythonVisitor::velocity, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position_ref", bp::make_function(&TaskJointPosturePythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_ref", bp::make_function(&TaskJointPosturePythonVisitor::velocity_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskJointPosturePythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskJointPosturePythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskJointPosturePythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskJointPosturePythonVisitor::setKd, bp::arg("Kd"))
        .def("compute", &TaskJointPosturePythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskJointPosturePythonVisitor::getConstraint)
        .add_property("name", &TaskJointPosturePythonVisitor::name)
        ;
      }
      static std::string name(TaskJoint & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskJoint & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskJoint & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static void setReference(TaskJoint & self, const trajectories::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskJoint & self){
        return self.getDesiredAcceleration();
      }
      static const Eigen::VectorXd & getmask(const TaskJoint & self){
        return self.mask();
      }
      static void setmask (TaskJoint & self, const Eigen::VectorXd mask){
        return self.mask(mask);
      }
      static Eigen::VectorXd getAcceleration (TaskJoint & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskJoint & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskJoint & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & position (const TaskJoint & self){
        return self.position();
      }
      static const Eigen::VectorXd & velocity (const TaskJoint & self){
        return self.velocity();
      }
      static const Eigen::VectorXd & position_ref (const TaskJoint & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & velocity_ref (const TaskJoint & self){
        return self.velocity_ref();
      }     
      static const Eigen::VectorXd & Kp (TaskJoint & self){
        return self.Kp();
      }  
      static const Eigen::VectorXd & Kd (TaskJoint & self){
        return self.Kd();
      }    
      static void setKp (TaskJoint & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskJoint & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskJoint info.";
        bp::class_<TaskJoint>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskJointPosturePythonVisitor<TaskJoint>());

      //  bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __tsid_python_task_joint_hpp__

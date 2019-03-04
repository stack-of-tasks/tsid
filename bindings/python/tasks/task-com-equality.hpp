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

#ifndef __tsid_python_task_com_hpp__
#define __tsid_python_task_com_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/tasks/task-com-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename TaskCOM>
    struct TaskCOMEqualityPythonVisitor
    : public boost::python::def_visitor< TaskCOMEqualityPythonVisitor<TaskCOM> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &> ((bp::arg("name"), bp::arg("robot")), "Default Constructor"))
        .add_property("dim", &TaskCOM::dim, "return dimension size")
        .def("setReference", &TaskCOMEqualityPythonVisitor::setReference, bp::arg("ref"))
        .add_property("getDesiredAcceleration", bp::make_function(&TaskCOMEqualityPythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .def("getAcceleration", &TaskCOMEqualityPythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskCOMEqualityPythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskCOMEqualityPythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position", bp::make_function(&TaskCOMEqualityPythonVisitor::position, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity", bp::make_function(&TaskCOMEqualityPythonVisitor::velocity, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position_ref", bp::make_function(&TaskCOMEqualityPythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_ref", bp::make_function(&TaskCOMEqualityPythonVisitor::velocity_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskCOMEqualityPythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskCOMEqualityPythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskCOMEqualityPythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskCOMEqualityPythonVisitor::setKd, bp::arg("Kd"))
        .def("compute", &TaskCOMEqualityPythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskCOMEqualityPythonVisitor::getConstraint)
        .add_property("name", &TaskCOMEqualityPythonVisitor::name)
        ;
      }
      static std::string name(TaskCOM & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskCOM & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskCOM & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static void setReference(TaskCOM & self, const trajectories::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskCOM & self){
        return self.getDesiredAcceleration();
      }
      static Eigen::VectorXd getAcceleration (TaskCOM & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskCOM & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskCOM & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & position (const TaskCOM & self){
        return self.position();
      }
      static const Eigen::VectorXd & velocity (const TaskCOM & self){
        return self.velocity();
      }
      static const Eigen::VectorXd & position_ref (const TaskCOM & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & velocity_ref (const TaskCOM & self){
        return self.velocity_ref();
      }     
      static const Eigen::Vector3d & Kp (TaskCOM & self){
        return self.Kp();
      }  
      static const Eigen::Vector3d & Kd (TaskCOM & self){
        return self.Kd();
      }    
      static void setKp (TaskCOM & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskCOM & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskCOMEqualityPythonVisitor info.";
        bp::class_<TaskCOM>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskCOMEqualityPythonVisitor<TaskCOM>());

        bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __tsid_python_task_com_hpp__

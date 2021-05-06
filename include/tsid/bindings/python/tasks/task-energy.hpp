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

#ifndef __tsid_python_task_energy_hpp__
#define __tsid_python_task_energy_hpp__

#include <pinocchio/fwd.hpp>
#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/tasks/task-energy.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename TaskEnergy>
    struct TaskEnergyPythonVisitor
    : public boost::python::def_visitor< TaskEnergyPythonVisitor<TaskEnergy> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, Eigen::VectorXd , Eigen::VectorXd , double, double> ((bp::arg("name"), bp::arg("robot"), bp::arg("position"), bp::arg("velocity"), bp::arg("dt"), bp::arg("timePreview")), "Default Constructor"))
        .add_property("dim", &TaskEnergy::dim, "return dimension size")
        .def("setReference", &TaskEnergyPythonVisitor::setReference, bp::arg("ref"))
        .add_property("position_ref", bp::make_function(&TaskEnergyPythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_BK_vector", bp::make_function(&TaskEnergyPythonVisitor::get_BK_vector, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_upperBound", bp::make_function(&TaskEnergyPythonVisitor::get_upperBound, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_E_c", bp::make_function(&TaskEnergyPythonVisitor::get_E_c, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_E_p", bp::make_function(&TaskEnergyPythonVisitor::get_E_p, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_E_tank", bp::make_function(&TaskEnergyPythonVisitor::get_E_tank, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_A", bp::make_function(&TaskEnergyPythonVisitor::get_A, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_upperBoundMaxEnergyCst", bp::make_function(&TaskEnergyPythonVisitor::get_upperBoundMaxEnergyCst, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_lowerBoundMaxEnergyCst", bp::make_function(&TaskEnergyPythonVisitor::get_lowerBoundMaxEnergyCst, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_vectorEnergyTask", bp::make_function(&TaskEnergyPythonVisitor::get_vectorEnergyTask, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_matrixEnergyTask", bp::make_function(&TaskEnergyPythonVisitor::get_matrixEnergyTask, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("get_LyapunovMatrix", bp::make_function(&TaskEnergyPythonVisitor::get_LyapunovMatrix, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("K", bp::make_function(&TaskEnergyPythonVisitor::K, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setK", &TaskEnergyPythonVisitor::setK, bp::arg("K"))
        .add_property("E_d", bp::make_function(&TaskEnergyPythonVisitor::E_d, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setE_d", &TaskEnergyPythonVisitor::setE_d, bp::arg("E"))
        .def("computeLyapunovTask", &TaskEnergyPythonVisitor::computeLyapunovTask, bp::args("t", "q", "v", "data"))
        .add_property("name", &TaskEnergyPythonVisitor::name)
        ;
      }
      static std::string name(TaskEnergy & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality computeLyapunovTask(TaskEnergy & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getLyapunovConstraint().name(), self.getLyapunovConstraint().matrix(), self.getLyapunovConstraint().vector());
        return cons;
      }
      static void setReference(TaskEnergy & self, const trajectories::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & position_ref (const TaskEnergy & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & get_BK_vector (const TaskEnergy & self){
        return self.get_BK_vector();
      }
      static const double & get_upperBound (const TaskEnergy & self){
        return self.get_upperBound();
      } 
      static const double & get_E_c (const TaskEnergy & self){
        return self.get_E_c();
      }
      static const double & get_E_p (const TaskEnergy & self){
        return self.get_E_p();
      }
      static const double & get_E_tank (const TaskEnergy & self){
        return self.get_E_tank();
      }
      static const Eigen::VectorXd & get_A (const TaskEnergy & self){
        return self.get_A();
      }
      static const double & get_upperBoundMaxEnergyCst (const TaskEnergy & self){
        return self.get_upperBoundMaxEnergyCst();
      } 
      static const double & get_lowerBoundMaxEnergyCst (const TaskEnergy & self){
        return self.get_lowerBoundMaxEnergyCst();
      } 
      static const double & get_vectorEnergyTask (const TaskEnergy & self){
        return self.get_vectorEnergyTask();
      } 
      static const Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> & get_matrixEnergyTask (const TaskEnergy & self){
        return self.get_matrixEnergyTask();
      }
      static const Eigen::Matrix<double, Eigen::Dynamic,Eigen::Dynamic> & get_LyapunovMatrix (const TaskEnergy & self){
        return self.get_LyapunovMatrix();
      } 
      static const Eigen::VectorXd & K (TaskEnergy & self){
        return self.K();
      }   
      static void setK (TaskEnergy & self, const::Eigen::VectorXd K){
        return self.K(K);
      }
      static const double & E_d (TaskEnergy & self){
        return self.E_d();
      }   
      static void setE_d (TaskEnergy & self, const double E){
        return self.setE_d(E);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskEnergy info.";
        bp::class_<TaskEnergy>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskEnergyPythonVisitor<TaskEnergy>());

      //  bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __tsid_python_task_energy_hpp__

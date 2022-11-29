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

#ifndef __tsid_python_task_joint_pos_vel_acc_bounds_hpp__
#define __tsid_python_task_joint_pos_vel_acc_bounds_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-joint-posVelAcc-bounds.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/constraint-bound.hpp"
#include "tsid/math/constraint-base.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename Task>
    struct TaskJointPosVelAccBoundsPythonVisitor
    : public boost::python::def_visitor< TaskJointPosVelAccBoundsPythonVisitor<Task> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, double> ((bp::arg("name"), bp::arg("robot"), bp::arg("Time step")), "Default Constructor"))
        .add_property("dim", &Task::dim, "return dimension size")
        .def("setTimeStep", &TaskJointPosVelAccBoundsPythonVisitor::setTimeStep, bp::args("dt"))
        .def("setPositionBounds", &TaskJointPosVelAccBoundsPythonVisitor::setPositionBounds, bp::args("lower", "upper"))
        .def("setVelocityBounds", &TaskJointPosVelAccBoundsPythonVisitor::setVelocityBounds, bp::args("upper"))
        .def("setAccelerationBounds", &TaskJointPosVelAccBoundsPythonVisitor::setAccelerationBounds, bp::args("upper"))
        .def("compute", &TaskJointPosVelAccBoundsPythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskJointPosVelAccBoundsPythonVisitor::getConstraint)
        .add_property("getPositionLowerBounds", bp::make_function(&TaskJointPosVelAccBoundsPythonVisitor::getVelocityUpperBounds, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("getUpperBounds", bp::make_function(&TaskJointPosVelAccBoundsPythonVisitor::getAccelerationUpperBounds, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("getAccelerationBounds", bp::make_function(&TaskJointPosVelAccBoundsPythonVisitor::getAccelerationLowerBounds, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("getVelocityBounds", bp::make_function(&TaskJointPosVelAccBoundsPythonVisitor::getVelocityLowerBounds, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("name", &TaskJointPosVelAccBoundsPythonVisitor::name)
        ;
      }
      static std::string name(Task & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintBound compute(Task & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, pinocchio::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintBound cons(self.getConstraint().name(),
                                   self.getConstraint().lowerBound(),
                                   self.getConstraint().upperBound());
        return cons;
      }
      static math::ConstraintBound getConstraint(const Task & self){
        math::ConstraintBound cons(self.getConstraint().name(),
                                   self.getConstraint().lowerBound(),
                                   self.getConstraint().upperBound());
        return cons;
      }
      static const Eigen::VectorXd & getAccelerationBounds (const Task & self){
        return self.getAccelerationBounds();
      }  
      static const Eigen::VectorXd & getVelocityBounds (const Task & self){
        return self.getVelocityBounds();
      }
      static const Eigen::VectorXd & getPositionLowerBounds (const Task & self){
        return self.getPositionLowerBounds();
      }
            static const Eigen::VectorXd & getPositionUpperBounds (const Task & self){
        return self.getPositionUpperBounds();
      }
      static void setTimeStep (Task & self, const double dt){
        return self.setTimeStep(dt);
      }
      static void setPositionBounds (Task & self, const Eigen::VectorXd lower, const Eigen::VectorXd upper){
        return self.setVelocityBounds(lower, upper);
      }
      static void setVelocityBounds (Task & self, const Eigen::VectorXd upper){
        return self.setVelocityBounds(upper);
      }
      static void setAccelerationBounds (Task & self, const Eigen::VectorXd upper){
        return self.setAccelerationBounds(upper);
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "Task info.";
        bp::class_<Task>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskJointPosVelAccBoundsPythonVisitor<Task>());
      }
    };
  }
}


#endif // ifndef __tsid_python_task_actuation_bounds_hpp__

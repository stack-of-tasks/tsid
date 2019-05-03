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

#ifndef __tsid_python_HQPOutput_hpp__
#define __tsid_python_HQPOutput_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/formulations/inverse-dynamics-formulation-acc-force.hpp"
#include "tsid/bindings/python/solvers/HQPData.hpp"
#include "tsid/contacts/contact-6d.hpp"
#include "tsid/contacts/contact-point.hpp"
#include "tsid/tasks/task-joint-posture.hpp"
#include "tsid/tasks/task-se3-equality.hpp"
#include "tsid/tasks/task-com-equality.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename T>
    struct InvDynPythonVisitor
    : public boost::python::def_visitor< InvDynPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, bool>((bp::args("name", "robot", "verbose"))))
        .def("data", &InvDynPythonVisitor::data)
        .add_property("nVar", &T::nVar)
        .add_property("nEq", &T::nEq)
        .add_property("nIn", &T::nIn)

        .def("addMotionTask", &InvDynPythonVisitor::addMotionTask_SE3, bp::args("task", "weight", "priorityLevel", "transition duration"))
        .def("addMotionTask", &InvDynPythonVisitor::addMotionTask_COM, bp::args("task", "weight", "priorityLevel", "transition duration"))
        .def("addMotionTask", &InvDynPythonVisitor::addMotionTask_Joint, bp::args("task", "weight", "priorityLevel", "transition duration"))
        
        
        .def("updateTaskWeight", &InvDynPythonVisitor::updateTaskWeight, bp::args("task_name", "weight"))
        .def("addRigidContact", &InvDynPythonVisitor::addRigidContact6dDeprecated, bp::args("contact"))
        .def("addRigidContact", &InvDynPythonVisitor::addRigidContact6d, bp::args("contact", "force_reg_weight"))
        .def("addRigidContact", &InvDynPythonVisitor::addRigidContactPoint, bp::args("contact", "force_reg_weight"))
        .def("addRigidContact", &InvDynPythonVisitor::addRigidContactPointWithPriorityLevel, bp::args("contact", "force_reg_weight", "motion_weight", "priority_level"))
        .def("removeTask", &InvDynPythonVisitor::removeTask, bp::args("task_name", "duration"))
        .def("removeRigidContact", &InvDynPythonVisitor::removeRigidContact, bp::args("contact_name", "duration"))
        .def("removeFromHqpData", &InvDynPythonVisitor::removeFromHqpData, bp::args("constraint_name"))
        .def("computeProblemData", &InvDynPythonVisitor::computeProblemData, bp::args("time", "q", "v"))
        
        .def("getActuatorForces", &InvDynPythonVisitor::getActuatorForces, bp::args("HQPOutput"))
        .def("getAccelerations", &InvDynPythonVisitor::getAccelerations, bp::args("HQPOutput"))
        .def("getContactForces", &InvDynPythonVisitor::getContactForces, bp::args("HQPOutput"))
        .def("checkContact", &InvDynPythonVisitor::checkContact, bp::args("name", "HQPOutput"))
        .def("getContactForce", &InvDynPythonVisitor::getContactForce, bp::args("name", "HQPOutput"))
        ;
      }
      static pinocchio::Data data(const T & self){
        pinocchio::Data data = self.data();
        return data;
      }
      static bool addMotionTask_SE3(T & self, tasks::TaskSE3Equality & task, double weight, unsigned int priorityLevel, double transition_duration){
        return self.addMotionTask(task, weight, priorityLevel, transition_duration);
      }
      static bool addMotionTask_COM(T & self, tasks::TaskComEquality & task, double weight, unsigned int priorityLevel, double transition_duration){
        return self.addMotionTask(task, weight, priorityLevel, transition_duration);
      }
      static bool addMotionTask_Joint(T & self, tasks::TaskJointPosture & task, double weight, unsigned int priorityLevel, double transition_duration){
        return self.addMotionTask(task, weight, priorityLevel, transition_duration);
      }
      static bool updateTaskWeight(T& self, const std::string & task_name, double weight){
        return self.updateTaskWeight(task_name, weight);
      }
      static bool addRigidContact6dDeprecated(T& self, contacts::Contact6d & contact){
        return self.addRigidContact(contact);
      }
      static bool addRigidContact6d(T& self, contacts::Contact6d & contact, double force_regularization_weight){
        return self.addRigidContact(contact, force_regularization_weight);
      }
      static bool addRigidContactPoint(T& self, contacts::ContactPoint & contact, double force_regularization_weight){
        return self.addRigidContact(contact, force_regularization_weight);
      }
      static bool addRigidContactPointWithPriorityLevel(T& self, contacts::ContactPoint & contact,
                                                        double force_regularization_weight,
                                                        double motion_weight,
                                                        const bool priority_level){
        return self.addRigidContact(contact, force_regularization_weight, motion_weight, priority_level);
      }
      static bool removeTask(T& self, const std::string & task_name, double transition_duration){
        return self.removeTask(task_name, transition_duration);
      }  
      static bool removeRigidContact(T& self, const std::string & contactName, double transition_duration){
        return self.removeRigidContact(contactName, transition_duration);
      }
      static bool removeFromHqpData(T& self, const std::string & constraintName){
        return self.removeFromHqpData(constraintName);
      }
      static HQPDatas computeProblemData(T& self, double time, const Eigen::VectorXd & q, const Eigen::VectorXd & v){
        HQPDatas Hqp;
        Hqp.set(self.computeProblemData(time, q, v));
        return Hqp;
      } 
      static Eigen::VectorXd getActuatorForces (T & self, const solvers::HQPOutput & sol){
        return self.getActuatorForces(sol);
      }
      static Eigen::VectorXd getAccelerations (T & self, const solvers::HQPOutput & sol){
        return self.getAccelerations(sol);
      }
      static Eigen::VectorXd getContactForces (T & self, const solvers::HQPOutput & sol){
        return self.getContactForces(sol);
      }
      static bool checkContact(T& self, const std::string & name,  const solvers::HQPOutput & sol){
          tsid::math::Vector f = self.getContactForces(name, sol);
          if(f.size()>0)
              return true;
          return false;
      }
      static Eigen::VectorXd getContactForce (T & self, const std::string & name, const solvers::HQPOutput & sol){
          return self.getContactForces(name, sol);
      }


      static void expose(const std::string & class_name)
      {
        std::string doc = "InvDyn info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(InvDynPythonVisitor<T>());       
      }
    };
  }
}


#endif // ifndef __tsid_python_HQPOutput_hpp__

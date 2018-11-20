#ifndef __tsid_python_task_se3_hpp__
#define __tsid_python_task_se3_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/tasks/task-se3-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;

    template<typename TaskSE3>
    struct TaskSE3PythonVisitor
    : public boost::python::def_visitor< TaskSE3PythonVisitor<TaskSE3> >
    {
      
      template<class PyClass>     
      

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, robots::RobotWrapper &, std::string> ((bp::arg("name"), bp::arg("robot"), bp::arg("framename")), "Default Constructor"))
        .add_property("dim", &TaskSE3::dim, "return dimension size")
        .def("setReference", &TaskSE3PythonVisitor::setReference, bp::arg("ref"))
        .add_property("getDesiredAcceleration", bp::make_function(&TaskSE3PythonVisitor::getDesiredAcceleration, bp::return_value_policy<bp::copy_const_reference>()), "Return Acc_desired")
        .def("getAcceleration", &TaskSE3PythonVisitor::getAcceleration, bp::arg("dv"))
        .add_property("position_error", bp::make_function(&TaskSE3PythonVisitor::position_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_error", bp::make_function(&TaskSE3PythonVisitor::velocity_error, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position", bp::make_function(&TaskSE3PythonVisitor::position, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity", bp::make_function(&TaskSE3PythonVisitor::velocity, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("position_ref", bp::make_function(&TaskSE3PythonVisitor::position_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("velocity_ref", bp::make_function(&TaskSE3PythonVisitor::velocity_ref, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kp", bp::make_function(&TaskSE3PythonVisitor::Kp, bp::return_value_policy<bp::copy_const_reference>()))
        .add_property("Kd", bp::make_function(&TaskSE3PythonVisitor::Kd, bp::return_value_policy<bp::copy_const_reference>()))
        .def("setKp", &TaskSE3PythonVisitor::setKp, bp::arg("Kp"))
        .def("setKd", &TaskSE3PythonVisitor::setKd, bp::arg("Kd"))
        .def("compute", &TaskSE3PythonVisitor::compute, bp::args("t", "q", "v", "data"))
        .def("getConstraint",  &TaskSE3PythonVisitor::getConstraint)
        .add_property("frame_id", &TaskSE3::frame_id, "frame id return")
        .add_property("name", &TaskSE3PythonVisitor::name)
        ;
      }
       static std::string name(TaskSE3 & self){
        std::string name = self.name();
        return name;
      }
      static math::ConstraintEquality compute(TaskSE3 & self, const double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, const se3::Data & data){
        self.compute(t, q, v, data);
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static math::ConstraintEquality getConstraint(const TaskSE3 & self){
        math::ConstraintEquality cons(self.getConstraint().name(), self.getConstraint().matrix(), self.getConstraint().vector());
        return cons;
      }
      static void setReference(TaskSE3 & self, trajectories::TrajectorySample & ref){
        self.setReference(ref);
      }
      static const Eigen::VectorXd & getDesiredAcceleration(const TaskSE3 & self){
        return self.getDesiredAcceleration();
      }
      static Eigen::VectorXd getAcceleration (TaskSE3 & self, const Eigen::VectorXd dv){
        return self.getAcceleration(dv);
      }
      static const Eigen::VectorXd & position_error(const TaskSE3 & self){
        return self.position_error();
      }
      static const Eigen::VectorXd & velocity_error(const TaskSE3 & self){
        return self.velocity_error();
      }
      static const Eigen::VectorXd & position (const TaskSE3 & self){
        return self.position();
      }
      static const Eigen::VectorXd & velocity (const TaskSE3 & self){
        return self.velocity();
      }
      static const Eigen::VectorXd & position_ref (const TaskSE3 & self){
        return self.position_ref();
      }
      static const Eigen::VectorXd & velocity_ref (const TaskSE3 & self){
        return self.velocity_ref();
      }     
      static const Eigen::VectorXd & Kp (TaskSE3 & self){
        return self.Kp();
      }  
      static const Eigen::VectorXd & Kd (TaskSE3 & self){
        return self.Kd();
      }    
      static void setKp (TaskSE3 & self, const::Eigen::VectorXd Kp){
        return self.Kp(Kp);
      }
      static void setKd (TaskSE3 & self, const::Eigen::VectorXd Kv){
        return self.Kd(Kv);
      }
      static Eigen::VectorXd frame_id (TaskSE3 & self){
        return self.frame_id();
      }
      static void expose(const std::string & class_name)
      {
        std::string doc = "TaskSE3 info.";
        bp::class_<TaskSE3>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TaskSE3PythonVisitor<TaskSE3>());

      //  bp::register_ptr_to_python< boost::shared_ptr<math::ConstraintBase> >();
      }
    };
  }
}


#endif // ifndef __tsid_python_task_se3_hpp__
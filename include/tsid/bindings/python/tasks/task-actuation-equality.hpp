/*
 * Author:  Ram Charan Teja Thota
 * Date:    August 1, 2024
 * 
 * Description:
 * this file contain boost python binding for task acutation equality 
 */


#ifndef __tsid_python_task_actuation_equality_hpp__
#define __tsid_python_task_actuation_equality_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/tasks/task-actuation-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/math/constraint-base.hpp"

namespace tsid{

namespace python{
namespace bp=boost::python;

template <typename Task>
struct TaskActuationEqualityPythonVisitor
   : public boost::python::def_visitor<
   TaskActuationEqualityPythonVisitor<Task>>{
    template <class PyClass>

    void visit (PyClass& cl) const{
        cl.def(bp::init<std::string, robots::RobotWrapper&>(
            (bp::arg("name")),bp::arg("robot")),"Default Constructor")
            .add_property("dim", &Task::dim, "return dimension size")
            .add_property("mask",
                      bp::make_function(
                          &TaskActuationEqualityPythonVisitor::getmask,
                          bp::return_value_policy<bp::copy_const_reference>()),
                      "Return mask")
            .def("setMask",&TaskActuationEqualityPythonVisitor::setmask,
             bp::arg("mask"))
            .def("compute", &TaskActuationEqualityPythonVisitor::compute,
             bp::args("t", "q", "v", "data"))
            .def
    }

  
  
  
 };






}  //namespace python
}  //namespace tsid


#endif // ifndef __tsid_python_task_actuation_equality_hpp__
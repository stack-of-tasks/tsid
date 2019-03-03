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

#ifndef __tsid_python_traj_se3_hpp__
#define __tsid_python_traj_se3_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/trajectories/trajectory-se3.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename TrajSE3>
    struct TrajectorySE3ConstantPythonVisitor
    : public boost::python::def_visitor< TrajectorySE3ConstantPythonVisitor<TrajSE3> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        .def(bp::init<std::string, pinocchio::SE3>((bp::arg("name"), bp::arg("reference")), "Default Constructor with name and ref_vec"))

        .add_property("size", &TrajSE3::size)
        .def("setReference", &TrajectorySE3ConstantPythonVisitor::setReference, bp::arg("M_ref"))
        .def("computeNext", &TrajectorySE3ConstantPythonVisitor::computeNext)
        .def("getLastSample", &TrajectorySE3ConstantPythonVisitor::getLastSample, bp::arg("sample"))
        .def("has_trajectory_ended", &TrajectorySE3ConstantPythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajectorySE3ConstantPythonVisitor::getSample, bp::arg("time"))
        ;
      }
      static void setReference(TrajSE3 & self, const pinocchio::SE3 & ref){
          self.setReference(ref);
      }
      static trajectories::TrajectorySample computeNext(TrajSE3 & self){
          return self.computeNext();
      }
      static void getLastSample(const TrajSE3 & self, trajectories::TrajectorySample & sample){
          self.getLastSample(sample);
      }
      static bool has_trajectory_ended(const TrajSE3 & self){
          return self.has_trajectory_ended();
      }
      static trajectories::TrajectorySample getSample(TrajSE3 & self, double time){
        return self.operator()(time);
      }
     

      static void expose(const std::string & class_name)
      {
        std::string doc = "Trajectory SE3 Constant info.";
        bp::class_<TrajSE3>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectorySE3ConstantPythonVisitor<TrajSE3>());
      }
    };
  }
}


#endif // ifndef __tsid_python_traj_se3_hpp__

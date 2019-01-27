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

#ifndef __tsid_python_traj_euclidian_hpp__
#define __tsid_python_traj_euclidian_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/trajectories/trajectory-euclidian.hpp"
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Traj>
    struct TrajectoryEuclidianConstantPythonVisitor
    : public boost::python::def_visitor< TrajectoryEuclidianConstantPythonVisitor<Traj> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        .def(bp::init<std::string, Eigen::VectorXd>((bp::arg("name"), bp::arg("reference")), "Default Constructor with name and ref_vec"))

        .add_property("size", &Traj::size)
        .def("setReference", &TrajectoryEuclidianConstantPythonVisitor::setReference, bp::arg("ref_vec"))
        .def("computeNext", &TrajectoryEuclidianConstantPythonVisitor::computeNext)
        .def("getLastSample", &TrajectoryEuclidianConstantPythonVisitor::getLastSample, bp::arg("sample"))
        .def("has_trajectory_ended", &TrajectoryEuclidianConstantPythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajectoryEuclidianConstantPythonVisitor::getSample, bp::arg("time"))
        ;
      }
      static void setReference(Traj & self, const Eigen::VectorXd & ref){
          self.setReference(ref);
      }
      static trajectories::TrajectorySample computeNext(Traj & self){
          return self.computeNext();
      }
      static void getLastSample(const Traj & self, trajectories::TrajectorySample & sample){
          self.getLastSample(sample);
      }
      static bool has_trajectory_ended(const Traj & self){
          return self.has_trajectory_ended();
      }
      static trajectories::TrajectorySample getSample(Traj & self, double time){
        return self.operator()(time);
      }
     

      static void expose(const std::string & class_name)
      {
        std::string doc = "Trajectory Euclidian Constant info.";
        bp::class_<Traj>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectoryEuclidianConstantPythonVisitor<Traj>());
      }
    };
  }
}


#endif // ifndef __tsid_python_traj_euclidian_hpp__
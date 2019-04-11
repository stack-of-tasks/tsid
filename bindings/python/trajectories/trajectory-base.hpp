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

#ifndef __tsid_python_traj_sample_hpp__
#define __tsid_python_traj_sample_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include <tsid/math/utils.hpp>
#include "tsid/trajectories/trajectory-base.hpp"
#include <assert.h>
namespace tsid
{
  namespace python
  {
    namespace bp = boost::python;
    typedef pinocchio::SE3 SE3;

    template<typename TrajSample>
    struct TrajectorySamplePythonVisitor
    : public boost::python::def_visitor< TrajectorySamplePythonVisitor<TrajSample> >
    {

      template<class PyClass>

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<unsigned int>((bp::arg("size")), "Default Constructor with size"))
        .def(bp::init<unsigned int, unsigned int>((bp::arg("pos_size"), bp::arg("vel_size")), "Default Constructor with pos and vel size"))

        .def("resize", &TrajectorySamplePythonVisitor::resize, bp::arg("size"))
        .def("resize", &TrajectorySamplePythonVisitor::resize2, bp::args("pos_size", "vel_size"))

        .def("pos", &TrajectorySamplePythonVisitor::pos)
        .def("vel", &TrajectorySamplePythonVisitor::vel)
        .def("acc", &TrajectorySamplePythonVisitor::acc)

        .def("pos", &TrajectorySamplePythonVisitor::setpos_vec)
        .def("pos", &TrajectorySamplePythonVisitor::setpos_se3)
        .def("vel", &TrajectorySamplePythonVisitor::setvel)
        .def("acc", &TrajectorySamplePythonVisitor::setacc)
        ;
      }

      static void setpos_vec(TrajSample & self, const Eigen::VectorXd pos){
        assert (self.pos.size() == pos.size());
        self.pos = pos;
      }
      static void setpos_se3(TrajSample & self, const pinocchio::SE3 & pos){
        assert (self.pos.size() == 12);
        tsid::math::SE3ToVector(pos, self.pos);
      }
      static void setvel(TrajSample & self, const Eigen::VectorXd vel){
        assert (self.vel.size() == vel.size());
        self.vel = vel;
      }
      static void setacc(TrajSample & self, const Eigen::VectorXd acc){
        assert (self.acc.size() == acc.size());
        self.acc = acc;
      }
      static void resize(TrajSample & self, const unsigned int & size){
          self.resize(size, size);
      }
      static void resize2(TrajSample & self, const unsigned int & pos_size, const unsigned int & vel_size){
          self.resize(pos_size, vel_size);
      }
      static Eigen::VectorXd pos(const TrajSample & self){
          return self.pos;
      }
      static Eigen::VectorXd vel(const TrajSample & self){
          return self.vel;
      }
      static Eigen::VectorXd acc(const TrajSample & self){
          return self.acc;
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Trajectory Sample info.";
        bp::class_<TrajSample>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajectorySamplePythonVisitor<TrajSample>());
      }
    };
  }
}


#endif // ifndef __tsid_python_traj_euclidian_hpp__
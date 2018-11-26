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


#ifndef __tsid_python_constriant_bound_hpp__
#define __tsid_python_constriant_bound_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/math/constraint-bound.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename ConstraintBound>
    struct ConstraintPythonVisitor
    : public boost::python::def_visitor< ConstraintPythonVisitor<ConstraintBound> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default constructor with name."))
        .def(bp::init<std::string, unsigned int>((bp::arg("name"), bp::arg("size")), "Default constructor with name and size."))
        .def(bp::init<std::string, Eigen::VectorXd, Eigen::VectorXd>((bp::arg("name"), bp::arg("lb"), bp::arg("ub")), "Default constructor with name and constraint."))
        
        .add_property("rows", &ConstraintBound::rows)
        .add_property("cols", &ConstraintBound::cols)
        .def("resize",&ConstraintBound::resize, (bp::arg("r"), bp::arg("c")),"Resize constraint size.")
        
        .add_property("isEquality", &ConstraintBound::isEquality)
        .add_property("isInequality", &ConstraintBound::isInequality)
        .add_property("isBound", &ConstraintBound::isBound)
        
        .add_property("vector", &ConstraintPythonVisitor::vector)
        .add_property("lowerBound", &ConstraintPythonVisitor::lowerBound)
        .add_property("upperBound", &ConstraintPythonVisitor::upperBound)

        .def("setVector", (bool (ConstraintBound::*)(const Eigen::VectorXd &) const) &ConstraintBound::setVector, bp::args("vector"), "Set Vector")
        .def("setLowerBound", (bool (ConstraintBound::*)(const Eigen::VectorXd &) const) &ConstraintBound::setLowerBound, bp::args("lb"), "Set LowerBound")
        .def("setUpperBound", (bool (ConstraintBound::*)(const Eigen::VectorXd &) const) &ConstraintBound::setUpperBound, bp::args("ub"), "Set UpperBound")
        ;
      }
      static Eigen::VectorXd vector (const ConstraintBound & self) {return self.vector();}
      static Eigen::VectorXd lowerBound (const ConstraintBound & self) {return self.lowerBound();}
      static Eigen::VectorXd upperBound (const ConstraintBound & self) {return self.upperBound();}
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Constraint Bound info.";
        bp::class_<ConstraintBound>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(ConstraintPythonVisitor<ConstraintBound>());
      }
    };
  }
}


#endif // ifndef __tsid_python_constriant_bound_hpp__
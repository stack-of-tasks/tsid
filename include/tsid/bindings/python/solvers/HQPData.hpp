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

#ifndef __tsid_python_test_hpp__
#define __tsid_python_test_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/bindings/python/utils/container.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename T>
    struct ConstPythonVisitor
    : public boost::python::def_visitor< ConstPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default Constructor"))
        .def("print_all", &T::print)
        .def("append", &T::append_eq, bp::arg("data"))
        .def("append", &T::append_ineq, bp::arg("data"))
        .def("append", &T::append_bound, bp::arg("data"))  
        ;
      }
       
      static void expose(const std::string & class_name)
      {
        std::string doc = "ConstraintLevel info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(ConstPythonVisitor<T>());       
      }
    };

    template<typename T>
    struct HQPPythonVisitor
    : public boost::python::def_visitor< HQPPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Default Constructor"))
        .def("print_all", &T::print)
        .def("resize", &T::resize, bp::arg("i"))
        .def("append", &T::append_helper, bp::arg("constraintLevel"))  
        ;
      }
       
      static void expose(const std::string & class_name)
      {
        std::string doc = "HQPdata info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(HQPPythonVisitor<T>());       
      }
    };

  }
}


#endif // ifndef __tsid_python_test_hpp__

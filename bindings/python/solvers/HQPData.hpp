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
        .def(bp::init<>("Defulat Constructor"))
        .def("print_all", &T::print)
        .def("append", &T::append_eq, bp::arg("data"))
        .def("append", &T::append_ineq, bp::arg("data"))
        .def("append", &T::append_bound, bp::arg("data"))  
        ;
      }
       
      static void expose(const std::string & class_name)
      {
        std::string doc = "ConstriantLevel info.";
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
        .def(bp::init<>("Defulat Constructor"))
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

#ifndef __tsid_python_HQPOutput_hpp__
#define __tsid_python_HQPOutput_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/solvers/solver-HQP-output.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename T>
    struct HQPOutputPythonVisitor
    : public boost::python::def_visitor< HQPOutputPythonVisitor<T> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<>("Defulat Constructor"))
        .def(bp::init<int, int, int>((bp::args("nVars", "nEq", "nInCon"))))
        .add_property("x", &HQPOutputPythonVisitor::x)
        ;
      }
      static Eigen::VectorXd x (const T & self) {return self.x;}
      static void expose(const std::string & class_name)
      {
        std::string doc = "HQPOutput info.";
        bp::class_<T>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(HQPOutputPythonVisitor<T>());       
      }
    };
  }
}


#endif // ifndef __tsid_python_HQPOutput_hpp__
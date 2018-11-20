#ifndef __tsid_python_solver_quadprog_hpp__
#define __tsid_python_solver_quadprog_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/solvers/solver-HQP-eiquadprog.hpp"
#include "tsid/solvers/solver-HQP-output.hpp"
#include "tsid/solvers/fwd.hpp"
#include "tsid/bindings/python/utils/container.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Solver>
    struct SolverQuadProgPythonVisitor
    : public boost::python::def_visitor< SolverQuadProgPythonVisitor<Solver> >
    {
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        
        .def("resize", &SolverQuadProgPythonVisitor::resize, bp::args("n", "neq", "nin"))
        .add_property("ObjVal", &Solver::getObjectiveValue, "retrun obj value")
        .def("solve", &SolverQuadProgPythonVisitor::solve, bp::args("HQPData"))
        .def("solve", &SolverQuadProgPythonVisitor::solver_helper, bp::args("HQPData for Python"))

        ;
      }
       
      static void resize(Solver & self, unsigned int n, unsigned int neq, unsigned int nin){
          self.resize(n, neq, nin);
      }  
      static solvers::HQPOutput solve(Solver & self, const solvers::HQPData & problemData){
          solvers::HQPOutput output;
          output = self.solve(problemData);
          return output;
      }
      static solvers::HQPOutput solver_helper(Solver & self, HQPDatas & HQPDatas){
          solvers::HQPOutput output;
          solvers::HQPData data = HQPDatas.get();

          output = self.solve(data);
         
          return output;
      }

      static void expose(const std::string & class_name)
      {
        std::string doc = "Solver EiQuadProg info.";
        bp::class_<Solver>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(SolverQuadProgPythonVisitor<Solver>());       
      }
    };
  }
}


#endif // ifndef __tsid_python_solver_quadprog_hpp__

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
    struct TrajSE3PythonVisitor
    : public boost::python::def_visitor< TrajSE3PythonVisitor<TrajSE3> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default Constructor with name"))
        .def(bp::init<std::string, se3::SE3>((bp::arg("name"), bp::arg("reference")), "Default Constructor with name and ref_vec"))

        .add_property("size", &TrajSE3::size)
        .def("setReference", &TrajSE3PythonVisitor::setReference, bp::arg("M_ref"))
        .def("computeNext", &TrajSE3PythonVisitor::computeNext)
        .def("getLastSample", &TrajSE3PythonVisitor::getLastSample, bp::arg("sample"))
        .def("has_trajectory_ended", &TrajSE3PythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajSE3PythonVisitor::getSample, bp::arg("time"))
        ;
      }
      static void setReference(TrajSE3 & self, const se3::SE3 & ref){
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
        std::string doc = "Traj SE3 Constant info.";
        bp::class_<TrajSE3>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajSE3PythonVisitor<TrajSE3>());
      }
    };
  }
}


#endif // ifndef __tsid_python_traj_se3_hpp__
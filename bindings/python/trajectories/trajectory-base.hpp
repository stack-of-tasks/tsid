#ifndef __tsid_python_traj_sample_hpp__
#define __tsid_python_traj_sample_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/trajectories/trajectory-base.hpp"
#include <assert.h>
namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename TrajSample>
    struct TrajSamplePythonVisitor
    : public boost::python::def_visitor< TrajSamplePythonVisitor<TrajSample> >
    {
      
      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<unsigned int>((bp::arg("size")), "Default Constructor with size"))
        .def(bp::init<unsigned int, unsigned int>((bp::arg("pos_size"), bp::arg("vel_size")), "Default Constructor with pos and vel size"))
        
        .def("resize", &TrajSamplePythonVisitor::resize, bp::arg("size"))
        .def("resize", &TrajSamplePythonVisitor::resize2, bp::args("pos_size", "vel_size"))
       
        .def("pos", &TrajSamplePythonVisitor::pos)
        .def("vel", &TrajSamplePythonVisitor::vel)
        .def("acc", &TrajSamplePythonVisitor::acc)

        .def("pos", &TrajSamplePythonVisitor::setpos)
        .def("vel", &TrajSamplePythonVisitor::setvel)
        .def("acc", &TrajSamplePythonVisitor::setacc)
        ;
      }
     
      static void setpos(TrajSample & self, const Eigen::VectorXd pos){
        assert (self.pos.size() == pos.size());
        self.pos = pos;
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
        std::string doc = "Traj Sample info.";
        bp::class_<TrajSample>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(TrajSamplePythonVisitor<TrajSample>());
      }
    };
  }
}


#endif // ifndef __tsid_python_traj_euclidian_hpp__
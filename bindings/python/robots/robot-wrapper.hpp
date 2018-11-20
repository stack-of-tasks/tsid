#ifndef __tsid_python_robot_wrapper_hpp__
#define __tsid_python_robot_wrapper_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename Robot>
    struct RobotPythonVisitor
    : public boost::python::def_visitor< RobotPythonVisitor<Robot> >
    {
      typedef typename Robot::std_vec std_vec;
      typedef math::Vector  Vector;
      typedef Eigen::Matrix<double,6,6> Matrix6d;
      typedef Eigen::Matrix<double,6,1> Vector6d;
      typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
      typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;


      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, std_vec, bool>((bp::arg("filename"), bp::arg("package_dir"), bp::arg("verbose")), "Default constructor without RootJoint."))
        .def(bp::init<std::string, std_vec, se3::JointModelVariant, bool>((bp::arg("filename"), bp::arg("package_dir"), bp::arg("roottype"), bp::arg("verbose")), "Default constructor without RootJoint."))
        .add_property("nq", &Robot::nq)
        .add_property("nv", &Robot::nv)
        
        .def("model", &RobotPythonVisitor::model)
        .def("data", &RobotPythonVisitor::data)

        .add_property("rotor_inertias", &RobotPythonVisitor::rotor_inertias)
        .add_property("gear_ratios", &RobotPythonVisitor::gear_ratios)
        .def("set_rotor_inertias", &RobotPythonVisitor::set_rotor_inertias, bp::arg("inertia vector"))
        .def("set_gear_ratios", &RobotPythonVisitor::set_gear_ratios, bp::arg("gear ratio vector"))
        
        .def("computeAllTerms", &RobotPythonVisitor::computeAllTerms, bp::args("data", "q", "v"), "compute all dynamics")
        .def("com", &RobotPythonVisitor::com, bp::arg("data"))
        .def("com_vel", &RobotPythonVisitor::com_vel, bp::arg("data"))
        .def("com_acc", &RobotPythonVisitor::com_acc, bp::arg("data"))
        .def("Jcom", &RobotPythonVisitor::Jcom, bp::arg("data"))
        .def("mass", &RobotPythonVisitor::mass, bp::arg("data")) 
        .def("nonLinearEffect", &RobotPythonVisitor::nonLinearEffects, bp::arg("data"))
        .def("position", &RobotPythonVisitor::position, bp::args("data", "index")) 
        .def("velocity", &RobotPythonVisitor::velocity, bp::args("data", "index"))
        .def("acceleration", &RobotPythonVisitor::acceleration, bp::args("data", "index"))
        
        .def("framePosition", &RobotPythonVisitor::framePosition, bp::args("data", "index"))
        .def("frameVelocity", &RobotPythonVisitor::frameVelocity, bp::args("data", "index"))
        .def("frameAcceleration", &RobotPythonVisitor::frameAcceleration, bp::args("data", "index"))
        .def("frameClassicAcceleration", &RobotPythonVisitor::frameClassicAcceleration, bp::args("data", "index"))
        
        //.def("jacobianWorld", &RobotPythonVisitor::jacobianWorld, bp::args("data", "index", "J"))
        //.def("jacobianLocal", &RobotPythonVisitor::jacobianLocal, bp::args("data", "index", "J")) // maybe these ftn are not used.
        //.def("framePosition", &RobotPythonVisitor::framePosition, bp::args("data", "index"))
        //.def("framePosition", &RobotPythonVisitor::framePosition, bp::args("data", "index", "framePosition"))
        
        ;
      }
      static se3::Model model (const Robot & self){
        return self.model();
      }
      static se3::Data data(const Robot & self){
        se3::Data data(self.model());
        return data;
      }
      static Eigen::VectorXd rotor_inertias(const Robot & self){
        return self.rotor_inertias();
      }
      static Eigen::VectorXd gear_ratios(const Robot & self){
        return self.gear_ratios();
      }
      static bool set_rotor_inertias(Robot & self, Eigen::VectorXd & rotor_inertias){
        return self.rotor_inertias(rotor_inertias);
      }
      static bool set_gear_ratios(Robot & self, Eigen::VectorXd & gear_ratios){
        return self.gear_ratios(gear_ratios);
      }

      static Eigen::Vector3d com (const Robot & self, const se3::Data & data){
        return self.com(data);
      }
      static Eigen::Vector3d com_vel (const Robot & self, const se3::Data & data){
        return self.com_vel(data);
      }
      static Eigen::Vector3d com_acc (const Robot & self, const se3::Data & data){
        return self.com_acc(data);
      } 
      static Matrix3x Jcom (const Robot & self, const se3::Data & data){
        return self.Jcom(data);
      } 
      static void computeAllTerms (const Robot & self, se3::Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v){
         self.computeAllTerms(data, q, v);
      }
      static Eigen::MatrixXd mass (Robot & self, se3::Data & data){
        return self.mass(data);
      }
      static Eigen::VectorXd nonLinearEffects(const Robot & self, const se3::Data & data){
        return self.nonLinearEffects(data);
      }
      static se3::SE3 position(const Robot & self, const se3::Data & data, const se3::Model::JointIndex & index){
        return self.position(data, index);
      }
      static se3::Motion velocity(const Robot & self, const se3::Data & data, const se3::Model::JointIndex & index){
        return self.velocity(data, index);
      }
      static se3::Motion acceleration(const Robot & self, const se3::Data & data, const se3::Model::JointIndex & index){
        return self.acceleration(data, index);
      }
      // static void jacobianWorld(const Robot & self, const se3::Data & data, const se3::Model::JointIndex & index, se3::Data::Matrix6x & J){
      //   self.jacobianWorld(data, index, J);
      // }
      // static void jacobianLocal(const Robot & self, const se3::Data & data, const se3::Model::JointIndex & index, se3::Data::Matrix6x & J){
      //   self.jacobianLocal(data, index, J);
      // }
      static se3::SE3 framePosition(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index){
        return self.framePosition(data, index);
      }
      //static void framePosition(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index, se3::SE3 & framePosition){
      //  self.framePosition(data, index, framePosition);
     // }
      static se3::Motion frameVelocity(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index){
        return self.frameVelocity(data, index);
      }
      // static void frameVelocity(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index, se3::Motion & frameVelocity){
      //   self.frameVelocity(data, index, frameVelocity);
      // }
      static se3::Motion frameAcceleration(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index){
        return self.frameAcceleration(data, index);
      }
      // static void frameAcceleration(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index, se3::Motion & frameAcceleration){
      //   self.frameAcceleration(data, index, frameAcceleration);
      // }
      static se3::Motion frameClassicAcceleration(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index){
        return self.frameClassicAcceleration(data, index);
      }
      // static void frameClassicAcceleration(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index, se3::Motion & frameAcceleration){
      //   self.frameClassicAcceleration(data, index, frameAcceleration);
      // }
      // static void frameJacobianWorld(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index, se3::Data::Matrix6x & J){
      //   self.frameJacobianWorld(data, index, J);
      // }
      // static void frameJacobianLocal(const Robot & self, const se3::Data & data, const se3::Model::FrameIndex & index, se3::Data::Matrix6x & J){
      //   self.frameJacobianLocal(data, index, J);
      // }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Robot Wrapper info.";
        bp::class_<Robot>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(RobotPythonVisitor<Robot>());
       // bp::class_< std::vector<std::string> >("StdVec_StdString")
       //   .def(bp::vector_indexing_suite< std::vector<std::string> >())
        ;
      }
    };
  }
}


#endif // ifndef __tsid_python_robot_wrapper_hpp__
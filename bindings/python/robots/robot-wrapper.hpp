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
      typedef std::vector<std::string> std_vec;
      typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;

      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string, std_vec, bool>((bp::arg("filename"), bp::arg("package_dir"), bp::arg("verbose")), "Default constructor without RootJoint."))
        .def(bp::init<pinocchio::Model, bool>((bp::arg("Pinocchio Model"), bp::arg("verbose")), "Default constructor from pinocchio model"))
        .def(bp::init<std::string, std_vec, pinocchio::JointModelVariant, bool>((bp::arg("filename"), bp::arg("package_dir"), bp::arg("roottype"), bp::arg("verbose")), "Default constructor without RootJoint."))
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
   
        ;
      }
      static pinocchio::Model model (const Robot & self){
        return self.model();
      }
      static pinocchio::Data data(const Robot & self){
        pinocchio::Data data(self.model());
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

      static Eigen::Vector3d com (const Robot & self, const pinocchio::Data & data){
        return self.com(data);
      }
      static Eigen::Vector3d com_vel (const Robot & self, const pinocchio::Data & data){
        return self.com_vel(data);
      }
      static Eigen::Vector3d com_acc (const Robot & self, const pinocchio::Data & data){
        return self.com_acc(data);
      } 
      static Matrix3x Jcom (const Robot & self, const pinocchio::Data & data){
        return self.Jcom(data);
      } 
      static void computeAllTerms (const Robot & self, pinocchio::Data & data, const Eigen::VectorXd & q, const Eigen::VectorXd & v){
         self.computeAllTerms(data, q, v);
      }
      static Eigen::MatrixXd mass (Robot & self, pinocchio::Data & data){
        return self.mass(data);
      }
      static Eigen::VectorXd nonLinearEffects(const Robot & self, const pinocchio::Data & data){
        return self.nonLinearEffects(data);
      }
      static pinocchio::SE3 position(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::JointIndex & index){
        return self.position(data, index);
      }
      static pinocchio::Motion velocity(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::JointIndex & index){
        return self.velocity(data, index);
      }
      static pinocchio::Motion acceleration(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::JointIndex & index){
        return self.acceleration(data, index);
      }
      static pinocchio::SE3 framePosition(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.framePosition(data, index);
      }
      static pinocchio::Motion frameVelocity(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.frameVelocity(data, index);
      }
      static pinocchio::Motion frameAcceleration(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.frameAcceleration(data, index);
      }
      static pinocchio::Motion frameClassicAcceleration(const Robot & self, const pinocchio::Data & data, const pinocchio::Model::FrameIndex & index){
        return self.frameClassicAcceleration(data, index);
      }
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Robot Wrapper info.";
        bp::class_<Robot>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(RobotPythonVisitor<Robot>());
        ;
      }
    };
  }
}


#endif // ifndef __tsid_python_robot_wrapper_hpp__

//
// Copyright (c) 2017 CNRS
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

#ifndef __invdyn_robot_wrapper_hpp__
#define __invdyn_robot_wrapper_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/robots/fwd.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/fwd.hpp>

#include <string>
#include <vector>


namespace tsid
{
  namespace robots
  {
    ///
    /// \brief Wrapper for a robot based on pinocchio
    ///
    class RobotWrapper
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Scalar Scalar;
      typedef pinocchio::Model Model;
      typedef pinocchio::Data Data;
      typedef pinocchio::Motion Motion;
      typedef pinocchio::Frame Frame;
      typedef pinocchio::SE3 SE3;
      typedef math::Vector  Vector;
      typedef math::Vector3 Vector3;
      typedef math::Vector6 Vector6;
      typedef math::Matrix Matrix;
      typedef math::Matrix3x Matrix3x;
      typedef math::RefVector RefVector;
      typedef math::ConstRefVector ConstRefVector;
      
      
      RobotWrapper(const std::string & filename,
                   const std::vector<std::string> & package_dirs,
                   bool verbose=false);

      RobotWrapper(const Model & m, bool verbose=false);

      RobotWrapper(const std::string & filename,
                   const std::vector<std::string> & package_dirs,
                   const pinocchio::JointModelVariant & rootJoint,
                   bool verbose=false);
      
      virtual int nq() const;
      virtual int nv() const;
      
      ///
      /// \brief Accessor to model.
      ///
      /// \returns a const reference on the model.
      ///
      const Model & model() const;
      Model & model();
      
      void computeAllTerms(Data & data, const Vector & q, const Vector & v) const;
      
      const Vector & rotor_inertias() const;
      const Vector & gear_ratios() const;
      
      bool rotor_inertias(ConstRefVector rotor_inertias);
      bool gear_ratios(ConstRefVector gear_ratios);
      
      void com(const Data & data,
               RefVector com_pos,
               RefVector com_vel,
               RefVector com_acc) const;
      
      const Vector3 & com(const Data & data) const;
      
      const Vector3 & com_vel(const Data & data) const;
      
      const Vector3 & com_acc(const Data & data) const;
      
      const Matrix3x & Jcom(const Data & data) const;
      
      const Matrix & mass(const Data & data);
      
      const Vector & nonLinearEffects(const Data & data) const;
      
      const SE3 & position(const Data & data,
                           const Model::JointIndex index) const;
      
      const Motion & velocity(const Data & data,
                              const Model::JointIndex index) const;
      
      const Motion & acceleration(const Data & data,
                                  const Model::JointIndex index) const;
      
      void jacobianWorld(const Data & data,
                         const Model::JointIndex index,
                         Data::Matrix6x & J) const;
      
      void jacobianLocal(const Data & data,
                         const Model::JointIndex index,
                         Data::Matrix6x & J) const;
      
      SE3 framePosition(const Data & data,
                        const Model::FrameIndex index) const;
      
      void framePosition(const Data & data,
                         const Model::FrameIndex index,
                         SE3 & framePosition) const;
      
      Motion frameVelocity(const Data & data,
                           const Model::FrameIndex index) const;
      
      void frameVelocity(const Data & data,
                         const Model::FrameIndex index,
                         Motion & frameVelocity) const;
      
      Motion frameAcceleration(const Data & data,
                               const Model::FrameIndex index) const;
      
      void frameAcceleration(const Data & data,
                             const Model::FrameIndex index,
                             Motion & frameAcceleration) const;
      
      Motion frameClassicAcceleration(const Data & data,
                                      const Model::FrameIndex index) const;
      
      void frameClassicAcceleration(const Data & data,
                                    const Model::FrameIndex index,
                                    Motion & frameAcceleration) const;
      
      void frameJacobianWorld(const Data & data,
                              const Model::FrameIndex index,
                              Data::Matrix6x & J) const;
      
      void frameJacobianLocal(const Data & data,
                              const Model::FrameIndex index,
                              Data::Matrix6x & J) const;
      
    protected:
      
      void updateMd();
      
      
      /// \brief Robot model.
      Model m_model;
      std::string m_model_filename;
      bool m_verbose;
      
      Vector m_rotor_inertias;
      Vector m_gear_ratios;
      Vector m_Md;  /// diagonal part of inertia matrix due to rotor inertias
      Matrix m_M;   /// inertia matrix including rotor inertias
    };
    
  } // namespace robots

} // namespace tsid

#endif // ifndef __invdyn_robot_wrapper_hpp__

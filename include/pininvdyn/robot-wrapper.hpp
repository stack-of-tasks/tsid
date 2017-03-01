//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_tasks_task_base_hpp__
#define __invdyn_tasks_task_base_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/spatial/skew.hpp"

#include <string>

namespace pininvdyn
{

  ///
  /// \brief Wrapper for a robot based on pinocchio
  ///
  class RobotWrapper
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef double Scalar;
    typedef se3::Model Model;
    typedef se3::Data Data;
    typedef se3::Motion Motion;
    typedef se3::Frame Frame;
    typedef se3::SE3 SE3;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    typedef Eigen::Matrix<Scalar,6,1> Vector6;
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;


    RobotWrapper(const std::string & filename,
                 const std::vector<std::string> & package_dirs,
                 bool verbose=false)
      : m_verbose(verbose)
    {
      se3::urdf::buildModel(filename, m_model, m_verbose);
      m_model_filename = filename;
    }

    RobotWrapper(const std::string & filename,
                 const std::vector<std::string> & package_dirs,
                 const se3::JointModelVariant & rootJoint,
                 bool verbose=false)
      : m_verbose(verbose)
    {
      se3::urdf::buildModel(filename, rootJoint, m_model, m_verbose);
      m_model_filename = filename;
    }

    virtual int nq() const { return m_model.nq; }
    virtual int nv() const { return m_model.nv; }

    ///
    /// \brief Accessor to model.
    ///
    /// \returns a const reference on the model.
    ///
    const Model & model() const { return m_model; }

    inline void computeAllTerms(Data & data, const Vector & q, const Vector & v) const
    {
      se3::computeAllTerms(m_model, data, q, v);
      se3::framesForwardKinematics(m_model, data);
    }

    inline const Vector3 & com(const Data & data) const
    {
      return data.com[0];
    }

    inline const Vector3 & com_vel(const Data & data) const
    {
      return data.vcom[0];
    }

    inline const Vector3 & com_acc(const Data & data) const
    {
      return data.acom[0];
    }

    inline const Matrix3x & Jcom(const Data & data) const
    {
      return data.Jcom;
    }

    inline const Matrix & mass(const Data & data) const
    {
      return data.M;
    }

    inline const Vector & nonLinearEffects(const Data & data) const
    {
      return data.nle;
    }

    inline const SE3 & position(const Data & data,
                                const Model::JointIndex index) const
    {
      return data.oMi[index];
    }

    inline const Motion & velocity(const Data & data,
                                   const Model::JointIndex index) const
    {
      return data.v[index];
    }

    inline const Motion & acceleration(const Data & data,
                                       const Model::JointIndex index) const
    {
      return data.a[index];
    }

    inline void jacobianWorld(const Data & data,
                              const Model::JointIndex index,
                              Data::Matrix6x & J) const
    {
      return se3::getJacobian<false>(m_model, data, index, J);
    }

    inline void jacobianLocal(const Data & data,
                              const Model::JointIndex index,
                              Data::Matrix6x & J) const
    {
      return se3::getJacobian<true>(m_model, data, index, J);
    }

    inline SE3 framePosition(const Data & data,
                             const Model::FrameIndex index) const
    {
      const Frame & f = m_model.frames[index];
      return data.oMi[f.parent].act(f.placement);
    }

    inline void framePosition(const Data & data,
                              const Model::FrameIndex index,
                              SE3 & framePosition) const
    {
      const Frame & f = m_model.frames[index];
      framePosition = data.oMi[f.parent].act(f.placement);
    }

    inline Motion frameVelocity(const Data & data,
                                const Model::FrameIndex index) const
    {
      const Frame & f = m_model.frames[index];
      return f.placement.actInv(data.v[f.parent]);
    }

    inline void frameVelocity(const Data & data,
                              const Model::FrameIndex index,
                              Motion & frameVelocity) const
    {
      const Frame & f = m_model.frames[index];
      frameVelocity = f.placement.actInv(data.v[f.parent]);
    }

    inline Motion frameAcceleration(const Data & data,
                                    const Model::FrameIndex index) const
    {
      const Frame & f = m_model.frames[index];
      return f.placement.actInv(data.a[f.parent]);
    }

    inline void frameAcceleration(const Data & data,
                                  const Model::FrameIndex index,
                                  Motion & frameAcceleration) const
    {
      const Frame & f = m_model.frames[index];
      frameAcceleration = f.placement.actInv(data.a[f.parent]);
    }

    inline Motion frameClassicAcceleration(const Data & data,
                                           const Model::FrameIndex index) const
    {
      const Frame & f = m_model.frames[index];
      Motion a = f.placement.actInv(data.a[f.parent]);
      Motion v = f.placement.actInv(data.v[f.parent]);
      a.linear() += v.angular().cross(v.linear());
      return a;
    }

    inline void frameClassicAcceleration(const Data & data,
                                         const Model::FrameIndex index,
                                         Motion & frameAcceleration) const
    {
      const Frame & f = m_model.frames[index];
      frameAcceleration = f.placement.actInv(data.a[f.parent]);
      Motion v = f.placement.actInv(data.v[f.parent]);
      frameAcceleration.linear() += v.angular().cross(v.linear());
    }

    inline void frameJacobianWorld(const Data & data,
                                   const Model::FrameIndex index,
                                   Data::Matrix6x & J) const
    {
      return se3::getFrameJacobian<false>(m_model, data, index, J);
    }

    inline void frameJacobianLocal(const Data & data,
                                   const Model::FrameIndex index,
                                   Data::Matrix6x & J) const
    {
      return se3::getFrameJacobian<true>(m_model, data, index, J);
    }

    //    const Vector3 & com(Data & data,const Vector & q,
    //                        const bool computeSubtreeComs = true,
    //                        const bool updateKinematics = true)
    //    {
    //      return se3::centerOfMass(m_model, data, q, computeSubtreeComs, updateKinematics);
    //    }
    //    const Vector3 & com(Data & data, const Vector & q, const Vector & v,
    //                 const bool computeSubtreeComs = true,
    //                 const bool updateKinematics = true)
    //    {
    //      return se3::centerOfMass(m_model, data, q, v, computeSubtreeComs, updateKinematics);
    //    }

  protected:


    /// \brief Robot model.
    Model m_model;
    std::string m_model_filename;
    bool m_verbose;
  };

}

#endif // ifndef __invdyn_tasks_task_base_hpp__

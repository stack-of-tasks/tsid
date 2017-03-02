//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#include <pininvdyn/robot-wrapper.hpp>
#include <pininvdyn/math/utils.hpp>

using namespace se3;
using namespace pininvdyn::math;

namespace pininvdyn
{

  RobotWrapper::RobotWrapper(const std::string & filename,
                             const std::vector<std::string> & package_dirs,
                             bool verbose)
    : m_verbose(verbose)
  {
    se3::urdf::buildModel(filename, m_model, m_verbose);
    m_model_filename = filename;
  }

  RobotWrapper::RobotWrapper(const std::string & filename,
                             const std::vector<std::string> & package_dirs,
                             const se3::JointModelVariant & rootJoint,
                             bool verbose)
    : m_verbose(verbose)
  {
    se3::urdf::buildModel(filename, rootJoint, m_model, m_verbose);
    m_model_filename = filename;
  }

  int RobotWrapper::nq() const { return m_model.nq; }
  int RobotWrapper::nv() const { return m_model.nv; }

  const Model & RobotWrapper::model() const { return m_model; }

  void RobotWrapper::computeAllTerms(Data & data, const Vector & q, const Vector & v) const
  {
    se3::computeAllTerms(m_model, data, q, v);
    se3::framesForwardKinematics(m_model, data);
  }

  const Vector3 & RobotWrapper::com(const Data & data) const
  {
    return data.com[0];
  }

  const Vector3 & RobotWrapper::com_vel(const Data & data) const
  {
    return data.vcom[0];
  }

  const Vector3 & RobotWrapper::com_acc(const Data & data) const
  {
    return data.acom[0];
  }

  const Matrix3x & RobotWrapper::Jcom(const Data & data) const
  {
    return data.Jcom;
  }

  const Matrix & RobotWrapper::mass(const Data & data) const
  {
    return data.M;
  }

  const Vector & RobotWrapper::nonLinearEffects(const Data & data) const
  {
    return data.nle;
  }

  const SE3 & RobotWrapper::position(const Data & data,
                                     const Model::JointIndex index) const
  {
    return data.oMi[index];
  }

  const Motion & RobotWrapper::velocity(const Data & data,
                                        const Model::JointIndex index) const
  {
    return data.v[index];
  }

  const Motion & RobotWrapper::acceleration(const Data & data,
                                            const Model::JointIndex index) const
  {
    return data.a[index];
  }

  void RobotWrapper::jacobianWorld(const Data & data,
                                   const Model::JointIndex index,
                                   Data::Matrix6x & J) const
  {
    return se3::getJacobian<false>(m_model, data, index, J);
  }

  void RobotWrapper::jacobianLocal(const Data & data,
                                   const Model::JointIndex index,
                                   Data::Matrix6x & J) const
  {
    return se3::getJacobian<true>(m_model, data, index, J);
  }

  SE3 RobotWrapper::framePosition(const Data & data,
                                  const Model::FrameIndex index) const
  {
    const Frame & f = m_model.frames[index];
    return data.oMi[f.parent].act(f.placement);
  }

  void RobotWrapper::framePosition(const Data & data,
                                   const Model::FrameIndex index,
                                   SE3 & framePosition) const
  {
    const Frame & f = m_model.frames[index];
    framePosition = data.oMi[f.parent].act(f.placement);
  }

  Motion RobotWrapper::frameVelocity(const Data & data,
                                     const Model::FrameIndex index) const
  {
    const Frame & f = m_model.frames[index];
    return f.placement.actInv(data.v[f.parent]);
  }

  void RobotWrapper::frameVelocity(const Data & data,
                                   const Model::FrameIndex index,
                                   Motion & frameVelocity) const
  {
    const Frame & f = m_model.frames[index];
    frameVelocity = f.placement.actInv(data.v[f.parent]);
  }

  Motion RobotWrapper::frameAcceleration(const Data & data,
                                         const Model::FrameIndex index) const
  {
    const Frame & f = m_model.frames[index];
    return f.placement.actInv(data.a[f.parent]);
  }

  void RobotWrapper::frameAcceleration(const Data & data,
                                       const Model::FrameIndex index,
                                       Motion & frameAcceleration) const
  {
    const Frame & f = m_model.frames[index];
    frameAcceleration = f.placement.actInv(data.a[f.parent]);
  }

  Motion RobotWrapper::frameClassicAcceleration(const Data & data,
                                                const Model::FrameIndex index) const
  {
    const Frame & f = m_model.frames[index];
    Motion a = f.placement.actInv(data.a[f.parent]);
    Motion v = f.placement.actInv(data.v[f.parent]);
    a.linear() += v.angular().cross(v.linear());
    return a;
  }

  void RobotWrapper::frameClassicAcceleration(const Data & data,
                                              const Model::FrameIndex index,
                                              Motion & frameAcceleration) const
  {
    const Frame & f = m_model.frames[index];
    frameAcceleration = f.placement.actInv(data.a[f.parent]);
    Motion v = f.placement.actInv(data.v[f.parent]);
    frameAcceleration.linear() += v.angular().cross(v.linear());
  }

  void RobotWrapper::frameJacobianWorld(const Data & data,
                                        const Model::FrameIndex index,
                                        Data::Matrix6x & J) const
  {
    return se3::getFrameJacobian<false>(m_model, data, index, J);
  }

  void RobotWrapper::frameJacobianLocal(const Data & data,
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

}

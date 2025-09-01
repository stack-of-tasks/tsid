//
// Copyright (c) 2025 CNRS INRIA LORIA
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

#include "tsid/contacts/measured-6d-wrench.hpp"

#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace contacts {

using namespace std;
using namespace math;
using namespace pinocchio;

Measured6Dwrench::Measured6Dwrench(const std::string& name, RobotWrapper& robot,
                                   const std::string& frameName)
    : MeasuredForceBase(name, robot), m_frame_name(frameName) {
  assert(m_robot.model().existFrame(frameName));
  m_frame_id = m_robot.model().getFrameId(frameName);

  m_fext.setZero();
  m_J.setZero(6, robot.nv());
  m_J_rotated.setZero(6, robot.nv());
  m_computedTorques.setZero(robot.nv());

  m_local_frame = true;
}

const Vector& Measured6Dwrench::computeJointTorques(Data& data) {
  m_robot.frameJacobianLocal(data, m_frame_id, m_J);

  if (!m_local_frame) {
    // Compute Jacobian in local world-oriented frame
    SE3 oMi, oMi_rotation_only;
    oMi_rotation_only.setIdentity();
    m_robot.framePosition(data, m_frame_id, oMi);
    oMi_rotation_only.rotation(oMi.rotation());

    // Use an explicit temporary `m_J_rotated` here to avoid allocations.
    m_J_rotated.noalias() = oMi_rotation_only.toActionMatrix() * m_J;
    m_J = m_J_rotated;
  }

  m_computedTorques = m_J.transpose() * m_fext;

  return m_computedTorques;
}

void Measured6Dwrench::setMeasuredContactForce(const Vector6& fext) {
  m_fext = fext;
}

const Vector6& Measured6Dwrench::getMeasuredContactForce() const {
  return m_fext;
}

void Measured6Dwrench::useLocalFrame(bool local_frame) {
  m_local_frame = local_frame;
}

}  // namespace contacts
}  // namespace tsid

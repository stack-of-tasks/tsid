//
// Copyright (c) 2025 CNRS INRIA LORIA
//

#include "tsid/contacts/measured-3d-force.hpp"

#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace contacts {

using namespace std;
using namespace math;
using namespace pinocchio;

typedef pinocchio::Data::Matrix6x Matrix6x;

Measured3Dforce::Measured3Dforce(const std::string& name, RobotWrapper& robot,
                                 const std::string& frameName)
    : MeasuredForceBase(name, robot), m_frame_name(frameName) {
  assert(m_robot.model().existFrame(frameName));
  m_frame_id = m_robot.model().getFrameId(frameName);

  m_fext.setZero();
  m_J.setZero(3, robot.nv());
  m_J_rotated.setZero(3, robot.nv());
  m_computedTorques.setZero(robot.nv());

  m_local_frame = true;
}

const Vector& Measured3Dforce::computeJointTorques(Data& data) {
  Matrix6x J;
  J.setZero(6, m_robot.nv());

  m_robot.frameJacobianLocal(data, m_frame_id, J);
  m_J = J.topRows(3);

  if (!m_local_frame) {
    // Compute Jacobian in local world-oriented frame
    SE3 oMi, oMi_rotation_only;
    oMi_rotation_only.setIdentity();
    m_robot.framePosition(data, m_frame_id, oMi);
    oMi_rotation_only.rotation(oMi.rotation());

    // Use an explicit temporary `m_J_rotated` here to avoid allocations.
    m_J_rotated.noalias() = (oMi_rotation_only.toActionMatrix() * J).topRows(3);
    m_J = m_J_rotated;
  }

  m_computedTorques = m_J.transpose() * m_fext;

  return m_computedTorques;
}

void Measured3Dforce::setMeasuredContactForce(const Vector3& fext) {
  m_fext = fext;
}

const Vector3& Measured3Dforce::getMeasuredContactForce() const {
  return m_fext;
}

void Measured3Dforce::useLocalFrame(bool local_frame) {
  m_local_frame = local_frame;
}

}  // namespace contacts
}  // namespace tsid

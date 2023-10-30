//
// Copyright (c) 2023 MIPT
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

#include "tsid/math/utils.hpp"
#include "tsid/tasks/task-two-frames-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace tasks {
using namespace std;
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskTwoFramesEquality::TaskTwoFramesEquality(const std::string& name,
                                             RobotWrapper& robot,
                                             const std::string& frameName1,
                                             const std::string& frameName2)
    : TaskMotion(name, robot),
      m_frame_name1(frameName1),
      m_frame_name2(frameName2),
      m_constraint(name, 6, robot.nv()) {
  assert(m_robot.model().existFrame(frameName1));
  assert(m_robot.model().existFrame(frameName2));
  m_frame_id1 = m_robot.model().getFrameId(frameName1);
  m_frame_id2 = m_robot.model().getFrameId(frameName2);

  m_v_ref.setZero();
  m_a_ref.setZero();
  m_wMl1.setIdentity();
  m_wMl2.setIdentity();
  m_p_error_vec.setZero(6);
  m_v_error_vec.setZero(6);
  m_p.resize(12);
  m_v.resize(6);
  m_p_ref.resize(12);
  m_v_ref_vec.resize(6);
  m_Kp.setZero(6);
  m_Kd.setZero(6);
  m_a_des.setZero(6);
  m_J1.setZero(6, robot.nv());
  m_J2.setZero(6, robot.nv());
  m_J1_rotated.setZero(6, robot.nv());
  m_J2_rotated.setZero(6, robot.nv());

  m_mask.resize(6);
  m_mask.fill(1.);
  setMask(m_mask);
}

void TaskTwoFramesEquality::setMask(math::ConstRefVector mask) {
  TaskMotion::setMask(mask);
  int n = dim();
  m_constraint.resize(n, (unsigned int)m_J1.cols());
  m_p_error_masked_vec.resize(n);
  m_v_error_masked_vec.resize(n);
  m_drift_masked.resize(n);
  m_a_des_masked.resize(n);
}

int TaskTwoFramesEquality::dim() const { return (int)m_mask.sum(); }

const Vector& TaskTwoFramesEquality::Kp() const { return m_Kp; }

const Vector& TaskTwoFramesEquality::Kd() const { return m_Kd; }

void TaskTwoFramesEquality::Kp(ConstRefVector Kp) {
  assert(Kp.size() == 6);
  m_Kp = Kp;
}

void TaskTwoFramesEquality::Kd(ConstRefVector Kd) {
  assert(Kd.size() == 6);
  m_Kd = Kd;
}

const Vector& TaskTwoFramesEquality::position_error() const {
  return m_p_error_masked_vec;
}

const Vector& TaskTwoFramesEquality::velocity_error() const {
  return m_v_error_masked_vec;
}

const Vector& TaskTwoFramesEquality::getDesiredAcceleration() const {
  return m_a_des_masked;
}

Vector TaskTwoFramesEquality::getAcceleration(ConstRefVector dv) const {
  return m_constraint.matrix() * dv + m_drift_masked;
}

Index TaskTwoFramesEquality::frame_id1() const { return m_frame_id1; }

Index TaskTwoFramesEquality::frame_id2() const { return m_frame_id2; }

const ConstraintBase& TaskTwoFramesEquality::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskTwoFramesEquality::compute(const double,
                                                     ConstRefVector,
                                                     ConstRefVector,
                                                     Data& data) {
  // Calculating task with formulation: [J1 - J2   0   0] y = [-J1dot*v +
  // J2dot*v]

  SE3 oMi1, oMi2;
  Motion v_frame1, v_frame2;
  Motion m_drift1, m_drift2;
  m_robot.framePosition(data, m_frame_id1, oMi1);
  m_robot.framePosition(data, m_frame_id2, oMi2);
  m_robot.frameVelocity(data, m_frame_id1, v_frame1);
  m_robot.frameVelocity(data, m_frame_id2, v_frame2);
  m_robot.frameClassicAcceleration(data, m_frame_id1, m_drift1);
  m_robot.frameClassicAcceleration(data, m_frame_id2, m_drift2);

  // Transformations from local to world
  m_wMl1.rotation(oMi1.rotation());
  m_wMl2.rotation(oMi2.rotation());

  m_robot.frameJacobianLocal(data, m_frame_id1, m_J1);
  m_robot.frameJacobianLocal(data, m_frame_id2, m_J2);

  // Doing all calculations in world frame
  errorInSE3(oMi1, oMi2, m_p_error);  // pos err in local (=rotated) oMi1 frame
  m_p_error_vec =
      m_wMl1.toActionMatrix() * m_p_error.toVector();  // pos err in world frame

  m_v_error =
      m_wMl2.act(v_frame2) - m_wMl1.act(v_frame1);  // vel err in world frame

  // desired acc in world frame
  m_a_des = m_Kp.cwiseProduct(m_p_error_vec) +
            m_Kd.cwiseProduct(m_v_error.toVector());

  m_v_error_vec = m_v_error.toVector();

  m_drift = (m_wMl1.act(m_drift1) - m_wMl2.act(m_drift2));

  m_J1_rotated.noalias() = m_wMl1.toActionMatrix() * m_J1;
  m_J1 = m_J1_rotated;

  m_J2_rotated.noalias() = m_wMl2.toActionMatrix() * m_J2;
  m_J2 = m_J2_rotated;

  int idx = 0;
  for (int i = 0; i < 6; i++) {
    if (m_mask(i) != 1.) continue;

    m_constraint.matrix().row(idx) = m_J1.row(i) - m_J2.row(i);
    m_constraint.vector().row(idx) = (m_a_des - m_drift.toVector()).row(i);
    m_a_des_masked(idx) = m_a_des(i);
    m_drift_masked(idx) = m_drift.toVector()(i);
    m_p_error_masked_vec(idx) = m_p_error_vec(i);
    m_v_error_masked_vec(idx) = m_v_error_vec(i);

    idx += 1;
  }

  return m_constraint;
}
}  // namespace tasks
}  // namespace tsid

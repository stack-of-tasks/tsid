//
// Copyright (c) 2017-2020 CNRS, NYU, MPI Tübingen, Inria
//

#include "tsid/math/utils.hpp"
#include "tsid/tasks/task-se3-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace tasks {
using namespace std;
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskSE3Equality::TaskSE3Equality(const std::string& name, RobotWrapper& robot,
                                 const std::string& frameName)
    : TaskMotion(name, robot),
      m_frame_name(frameName),
      m_constraint(name, 6, robot.nv()),
      m_ref(12, 6) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      m_robot.model().existFrame(frameName),
      "The frame with name '" + frameName + "' does not exist");
  m_frame_id = m_robot.model().getFrameId(frameName);

  m_v_ref.setZero();
  m_a_ref.setZero();
  m_M_ref.setIdentity();
  m_wMl.setIdentity();
  m_p_error_vec.setZero(6);
  m_v_error_vec.setZero(6);
  m_p.resize(12);
  m_v.resize(6);
  m_p_ref.resize(12);
  m_v_ref_vec.resize(6);
  m_Kp.setZero(6);
  m_Kd.setZero(6);
  m_a_des.setZero(6);
  m_J.setZero(6, robot.nv());
  m_J_rotated.setZero(6, robot.nv());

  m_mask.resize(6);
  m_mask.fill(1.);
  setMask(m_mask);

  m_local_frame = true;
}

void TaskSE3Equality::setMask(math::ConstRefVector mask) {
  TaskMotion::setMask(mask);
  int n = dim();
  m_constraint.resize(n, (unsigned int)m_J.cols());
  m_p_error_masked_vec.resize(n);
  m_v_error_masked_vec.resize(n);
  m_drift_masked.resize(n);
  m_a_des_masked.resize(n);
}

int TaskSE3Equality::dim() const { return (int)m_mask.sum(); }

const Vector& TaskSE3Equality::Kp() const { return m_Kp; }

const Vector& TaskSE3Equality::Kd() const { return m_Kd; }

void TaskSE3Equality::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == 6,
                                 "The size of the Kp vector needs to equal 6");
  m_Kp = Kp;
}

void TaskSE3Equality::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == 6,
                                 "The size of the Kd vector needs to equal 6");
  m_Kd = Kd;
}

void TaskSE3Equality::setReference(TrajectorySample& ref) {
  m_ref = ref;
  TSID_DISABLE_WARNING_PUSH
  TSID_DISABLE_WARNING_DEPRECATED
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      ref.pos.size() == 12, "The size of the reference vector needs to be 12");
  m_M_ref.translation(ref.pos.head<3>());
  m_M_ref.rotation(MapMatrix3(&ref.pos(3), 3, 3));
  TSID_DISABLE_WARNING_POP
  m_v_ref = Motion(ref.getDerivative());
  m_a_ref = Motion(ref.getSecondDerivative());
}

void TaskSE3Equality::setReference(const SE3& ref) {
  TrajectorySample s(12, 6);
  TSID_DISABLE_WARNING_PUSH
  TSID_DISABLE_WARNING_DEPRECATED
  tsid::math::SE3ToVector(ref, s.pos);
  TSID_DISABLE_WARNING_POP
  setReference(s);
}

const TrajectorySample& TaskSE3Equality::getReference() const { return m_ref; }

const Vector& TaskSE3Equality::position_error() const {
  return m_p_error_masked_vec;
}

const Vector& TaskSE3Equality::velocity_error() const {
  return m_v_error_masked_vec;
}

const Vector& TaskSE3Equality::position() const { return m_p; }

const Vector& TaskSE3Equality::velocity() const { return m_v; }

const Vector& TaskSE3Equality::position_ref() const { return m_p_ref; }

const Vector& TaskSE3Equality::velocity_ref() const { return m_v_ref_vec; }

const Vector& TaskSE3Equality::getDesiredAcceleration() const {
  return m_a_des_masked;
}

Vector TaskSE3Equality::getAcceleration(ConstRefVector dv) const {
  return m_constraint.matrix() * dv + m_drift_masked;
}

Index TaskSE3Equality::frame_id() const { return m_frame_id; }

const ConstraintBase& TaskSE3Equality::getConstraint() const {
  return m_constraint;
}

void TaskSE3Equality::useLocalFrame(bool local_frame) {
  m_local_frame = local_frame;
}

const ConstraintBase& TaskSE3Equality::compute(const double, ConstRefVector,
                                               ConstRefVector, Data& data) {
  SE3 oMi;
  Motion v_frame;
  m_robot.framePosition(data, m_frame_id, oMi);
  m_robot.frameVelocity(data, m_frame_id, v_frame);
  m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);

  // @todo Since Jacobian computation is cheaper in world frame
  // we could do all computations in world frame
  m_robot.frameJacobianLocal(data, m_frame_id, m_J);

  errorInSE3(oMi, m_M_ref, m_p_error);  // pos err in local frame
  SE3ToVector(m_M_ref, m_p_ref);
  SE3ToVector(oMi, m_p);

  // Transformation from local to world
  m_wMl.rotation(oMi.rotation());

  if (m_local_frame) {
    m_p_error_vec = m_p_error.toVector();
    m_v_error = m_wMl.actInv(m_v_ref) - v_frame;  // vel err in local frame

    // desired acc in local frame
    m_a_des = m_Kp.cwiseProduct(m_p_error_vec) +
              m_Kd.cwiseProduct(m_v_error.toVector()) +
              m_wMl.actInv(m_a_ref).toVector();
  } else {
    m_p_error_vec =
        m_wMl.toActionMatrix() *  // pos err in local world-oriented frame
        m_p_error.toVector();

    // cout<<"m_p_error_vec="<<m_p_error_vec.head<3>().transpose()<<endl;
    // cout<<"oMi-m_M_ref
    // ="<<-(oMi.translation()-m_M_ref.translation()).transpose()<<endl;

    m_v_error =
        m_v_ref - m_wMl.act(v_frame);  // vel err in local world-oriented frame

    m_drift = m_wMl.act(m_drift);

    // desired acc in local world-oriented frame
    m_a_des = m_Kp.cwiseProduct(m_p_error_vec) +
              m_Kd.cwiseProduct(m_v_error.toVector()) + m_a_ref.toVector();

    // Use an explicit temporary `m_J_rotated` here to avoid allocations.
    m_J_rotated.noalias() = m_wMl.toActionMatrix() * m_J;
    m_J = m_J_rotated;
  }

  m_v_error_vec = m_v_error.toVector();
  m_v_ref_vec = m_v_ref.toVector();
  m_v = v_frame.toVector();

  int idx = 0;
  for (int i = 0; i < 6; i++) {
    if (m_mask(i) != 1.) continue;

    m_constraint.matrix().row(idx) = m_J.row(i);
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

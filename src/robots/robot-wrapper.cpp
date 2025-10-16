//
// Copyright (c) 2017 CNRS
//

#include "tsid/robots/robot-wrapper.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

using namespace pinocchio;
using namespace tsid::math;

namespace tsid {
namespace robots {

RobotWrapper::RobotWrapper(const std::string& filename,
                           const std::vector<std::string>&, bool verbose)
    : m_verbose(verbose) {
  pinocchio::urdf::buildModel(filename, m_model, m_verbose);
  m_model_filename = filename;
  m_na = m_model.nv;
  m_nq_actuated = m_model.nq;
  m_is_fixed_base = true;
  init();
}

RobotWrapper::RobotWrapper(const std::string& filename,
                           const std::vector<std::string>&,
                           const pinocchio::JointModelVariant& rootJoint,
                           bool verbose)
    : m_verbose(verbose) {
  pinocchio::urdf::buildModel(filename, rootJoint, m_model, m_verbose);
  m_model_filename = filename;
  m_na = m_model.nv - 6;
  m_nq_actuated = m_model.nq - 7;
  m_is_fixed_base = false;
  init();
}

RobotWrapper::RobotWrapper(const pinocchio::Model& m, bool verbose)
    : m_verbose(verbose) {
  m_model = m;
  m_model_filename = "";
  m_na = m_model.nv - 6;
  m_nq_actuated = m_model.nq - 7;
  m_is_fixed_base = false;
  init();
}

RobotWrapper::RobotWrapper(const pinocchio::Model& m, RootJointType rootJoint,
                           bool verbose)
    : m_verbose(verbose) {
  m_model = m;
  m_model_filename = "";
  m_na = m_model.nv;
  m_nq_actuated = m_model.nq;
  m_is_fixed_base = true;
  switch (rootJoint) {
    case FIXED_BASE_SYSTEM:
      break;
    case FLOATING_BASE_SYSTEM:
      m_na -= 6;
      m_nq_actuated = m_model.nq - 7;
      m_is_fixed_base = false;
    default:
      break;
  }
  init();
}

void RobotWrapper::init() {
  m_rotor_inertias.setZero(m_na);
  m_gear_ratios.setZero(m_na);
  m_Md.setZero(m_na);
  m_M.setZero(m_model.nv, m_model.nv);
}

int RobotWrapper::nq() const { return m_model.nq; }
int RobotWrapper::nv() const { return m_model.nv; }
int RobotWrapper::na() const { return m_na; }
int RobotWrapper::nq_actuated() const { return m_nq_actuated; }
bool RobotWrapper::is_fixed_base() const { return m_is_fixed_base; }

const Model& RobotWrapper::model() const { return m_model; }
Model& RobotWrapper::model() { return m_model; }

void RobotWrapper::computeAllTerms(Data& data, const Vector& q,
                                   const Vector& v) const {
  pinocchio::computeAllTerms(m_model, data, q, v);
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
  // computeAllTerms does not compute the com acceleration, so we need to call
  // centerOfMass Check this line, calling with zero acceleration at the last
  // phase compute the CoM acceleration.
  //      pinocchio::centerOfMass(m_model, data, q,v,false);
  pinocchio::updateFramePlacements(m_model, data);
  pinocchio::centerOfMass(m_model, data, q, v, Eigen::VectorXd::Zero(nv()));
  pinocchio::ccrba(m_model, data, q, v);
}

const Vector& RobotWrapper::rotor_inertias() const { return m_rotor_inertias; }
const Vector& RobotWrapper::gear_ratios() const { return m_gear_ratios; }

bool RobotWrapper::rotor_inertias(ConstRefVector rotor_inertias) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      rotor_inertias.size() == m_rotor_inertias.size(),
      "The size of the rotor_inertias vector is incorrect!");
  m_rotor_inertias = rotor_inertias;
  updateMd();
  return true;
}

bool RobotWrapper::gear_ratios(ConstRefVector gear_ratios) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      gear_ratios.size() == m_gear_ratios.size(),
      "The size of the gear_ratios vector is incorrect!");
  m_gear_ratios = gear_ratios;
  updateMd();
  return true;
}

void RobotWrapper::updateMd() {
  m_Md =
      m_gear_ratios.cwiseProduct(m_gear_ratios.cwiseProduct(m_rotor_inertias));
}

void RobotWrapper::com(const Data& data, RefVector com_pos, RefVector com_vel,
                       RefVector com_acc) const {
  com_pos = data.com[0];
  com_vel = data.vcom[0];
  com_acc = data.acom[0];
}

const Vector3& RobotWrapper::com(const Data& data) const { return data.com[0]; }

const Vector3& RobotWrapper::com_vel(const Data& data) const {
  return data.vcom[0];
}

const Vector3& RobotWrapper::com_acc(const Data& data) const {
  return data.acom[0];
}

const Matrix3x& RobotWrapper::Jcom(const Data& data) const { return data.Jcom; }

const Matrix& RobotWrapper::mass(const Data& data) {
  m_M = data.M;
  m_M.diagonal().tail(m_na) += m_Md;
  return m_M;
}

const Vector& RobotWrapper::nonLinearEffects(const Data& data) const {
  return data.nle;
}

const SE3& RobotWrapper::position(const Data& data,
                                  const Model::JointIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.oMi.size(),
      "The index needs to be less than the size of the oMi vector");
  return data.oMi[index];
}

const Motion& RobotWrapper::velocity(const Data& data,
                                     const Model::JointIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.v.size(),
      "The index needs to be less than the size of the v vector");
  return data.v[index];
}

const Motion& RobotWrapper::acceleration(const Data& data,
                                         const Model::JointIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.a.size(),
      "The index needs to be less than the size of the a vector");
  return data.a[index];
}

void RobotWrapper::jacobianWorld(const Data& data,
                                 const Model::JointIndex index,
                                 Data::Matrix6x& J) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.oMi.size(),
      "The index needs to be less than the size of the oMi vector");
  return pinocchio::getJointJacobian(m_model, data, index, pinocchio::WORLD, J);
}

void RobotWrapper::jacobianLocal(const Data& data,
                                 const Model::JointIndex index,
                                 Data::Matrix6x& J) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.oMi.size(),
      "The index needs to be less than the size of the oMi vector");
  return pinocchio::getJointJacobian(m_model, data, index, pinocchio::LOCAL, J);
}

SE3 RobotWrapper::framePosition(const Data& data,
                                const Model::FrameIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  return data.oMi[f.parent].act(f.placement);
}

void RobotWrapper::framePosition(const Data& data,
                                 const Model::FrameIndex index,
                                 SE3& framePosition) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  framePosition = data.oMi[f.parent].act(f.placement);
}

Motion RobotWrapper::frameVelocity(const Data& data,
                                   const Model::FrameIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  return f.placement.actInv(data.v[f.parent]);
}

void RobotWrapper::frameVelocity(const Data& data,
                                 const Model::FrameIndex index,
                                 Motion& frameVelocity) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  frameVelocity = f.placement.actInv(data.v[f.parent]);
}

Motion RobotWrapper::frameVelocityWorldOriented(
    const Data& data, const Model::FrameIndex index) const {
  Motion v_local, v_world;
  SE3 oMi, oMi_rotation_only;
  framePosition(data, index, oMi);
  frameVelocity(data, index, v_local);
  oMi_rotation_only.rotation(oMi.rotation());
  v_world = oMi_rotation_only.act(v_local);
  return v_world;
}

Motion RobotWrapper::frameAcceleration(const Data& data,
                                       const Model::FrameIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  return f.placement.actInv(data.a[f.parent]);
}

void RobotWrapper::frameAcceleration(const Data& data,
                                     const Model::FrameIndex index,
                                     Motion& frameAcceleration) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  frameAcceleration = f.placement.actInv(data.a[f.parent]);
}

Motion RobotWrapper::frameAccelerationWorldOriented(
    const Data& data, const Model::FrameIndex index) const {
  Motion a_local, a_world;
  SE3 oMi, oMi_rotation_only;
  framePosition(data, index, oMi);
  frameAcceleration(data, index, a_local);
  oMi_rotation_only.rotation(oMi.rotation());
  a_world = oMi_rotation_only.act(a_local);
  return a_world;
}

Motion RobotWrapper::frameClassicAcceleration(
    const Data& data, const Model::FrameIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  Motion a = f.placement.actInv(data.a[f.parent]);
  Motion v = f.placement.actInv(data.v[f.parent]);
  a.linear() += v.angular().cross(v.linear());
  return a;
}

void RobotWrapper::frameClassicAcceleration(const Data& data,
                                            const Model::FrameIndex index,
                                            Motion& frameAcceleration) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame& f = m_model.frames[index];
  frameAcceleration = f.placement.actInv(data.a[f.parent]);
  Motion v = f.placement.actInv(data.v[f.parent]);
  frameAcceleration.linear() += v.angular().cross(v.linear());
}

Motion RobotWrapper::frameClassicAccelerationWorldOriented(
    const Data& data, const Model::FrameIndex index) const {
  Motion a_local, a_world;
  SE3 oMi, oMi_rotation_only;
  framePosition(data, index, oMi);
  frameClassicAcceleration(data, index, a_local);
  oMi_rotation_only.rotation(oMi.rotation());
  a_world = oMi_rotation_only.act(a_local);
  return a_world;
}

void RobotWrapper::frameJacobianWorld(Data& data, const Model::FrameIndex index,
                                      Data::Matrix6x& J) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  return pinocchio::getFrameJacobian(m_model, data, index, pinocchio::WORLD, J);
}

void RobotWrapper::frameJacobianLocal(Data& data, const Model::FrameIndex index,
                                      Data::Matrix6x& J) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  return pinocchio::getFrameJacobian(m_model, data, index, pinocchio::LOCAL, J);
}

const Data::Matrix6x& RobotWrapper::momentumJacobian(const Data& data) const {
  return data.Ag;
}

Vector3 RobotWrapper::angularMomentumTimeVariation(const Data& data) const {
  return pinocchio::computeCentroidalMomentumTimeVariation(
             m_model, const_cast<Data&>(data))
      .angular();
}

void RobotWrapper::setGravity(const Motion& gravity) {
  m_model.gravity = gravity;
}

//    const Vector3 & com(Data & data,const Vector & q,
//                        const bool computeSubtreeComs = true,
//                        const bool updateKinematics = true)
//    {
//      return pinocchio::centerOfMass(m_model, data, q, computeSubtreeComs,
//      updateKinematics);
//    }
//    const Vector3 & com(Data & data, const Vector & q, const Vector & v,
//                 const bool computeSubtreeComs = true,
//                 const bool updateKinematics = true)
//    {
//      return pinocchio::centerOfMass(m_model, data, q, v, computeSubtreeComs,
//      updateKinematics);
//    }

}  // namespace robots
}  // namespace tsid

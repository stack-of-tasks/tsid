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

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include "tsid/tasks/task-contact-force-equality.hpp"

using namespace tsid::math;
using namespace std;

namespace tsid {
namespace tasks {


TaskContactForceEquality::TaskContactForceEquality(const std::string & name, RobotWrapper & robot,
                                                   const std::string & contactName):
  TaskContactForce(name, robot),
  m_contact_name(contactName),
  m_constraint(name, 6, 6),
  m_ref(6,6),
  m_fext(6,6) {
  m_forceIntegralError = Vector::Zero(6);
  m_dt = 0.001;
}

void TaskContactForceEquality::setContactList(const std::vector<std::shared_ptr<ContactLevel> >  *contacts) {
  m_contacts = contacts;
}

int TaskContactForceEquality::dim() const {
  return 6;
}

const Vector & TaskContactForceEquality::Kp() const { return m_Kp; }
const Vector & TaskContactForceEquality::Kd() const { return m_Kd; }
const Vector & TaskContactForceEquality::Ki() const { return m_Ki; }

void TaskContactForceEquality::Kp(ConstRefVector Kp)
{
  assert(Kp.size()==6);
  m_Kp = Kp;
}

void TaskContactForceEquality::Kd(ConstRefVector Kd)
{
  assert(Kd.size()==6);
  m_Kd = Kd;
}

void TaskContactForceEquality::Ki(ConstRefVector Ki)
{
  assert(Ki.size()==6);
  m_Ki = Ki;
}

const std::string& TaskContactForceEquality::getAssociatedContactName() {
  return m_contact_name;
}

void TaskContactForceEquality::setAssociatedContactName(const std::string & contactName) {
  m_contact_name = contactName;
}

void TaskContactForceEquality::setReference(TrajectorySample & ref) {
  m_ref = ref;
}

const TaskContactForceEquality::TrajectorySample & TaskContactForceEquality::getReference() const {
  return m_ref;
}

void TaskContactForceEquality::setExternalForce(TrajectorySample & f_ext) {
  m_fext = f_ext;
}

const TaskContactForceEquality::TrajectorySample & TaskContactForceEquality::getExternalForce() const {
  return m_fext;
}


const ConstraintBase & TaskContactForceEquality::compute(const double t,
                                                         ConstRefVector q,
                                                         ConstRefVector v,
                                                         Data & data,
                                                         const std::vector<std::shared_ptr<ContactLevel> >  *contacts) {
  setContactList(contacts);
  return compute(t, q, v, data);
}
const ConstraintBase & TaskContactForceEquality::compute(const double,
                                                         ConstRefVector,
                                                         ConstRefVector,
                                                         Data & data) {

  std::cout << "################### task contact force compute ###################" << std::endl;
  // fill constraint matrix
  SE3 oMi;
  // Vector3 p_local, p_world;
  auto& M = m_constraint.matrix();
  std::shared_ptr<ContactLevel> cl = (*m_contacts)[0];
  int n = cl->contact.n_force(); //6
  M.resize(6, n);
  // get frame of the contactPoint
  // m_robot.framePosition(data, cl->contact.getMotionTask().frame_id(), oMi);
  const Matrix & T = cl->contact.getForceGeneratorMatrix(); // e.g., 6x12 for a 6d contact
  M = T.transpose()*cl->contact.getMotionConstraint().matrix();
  std::cout << "################### task contact force M ok ###################" << std::endl;

  const Vector6 forceError = m_ref.pos - m_fext.pos;
  m_constraint.vector() = m_Kp.cwiseProduct(forceError) + m_Kd.cwiseProduct(m_ref.vel - m_fext.vel) 
                          + m_Ki.cwiseProduct(m_forceIntegralError);
  std::cout << "################### task contact force m_constraint ok ###################" << std::endl;

  m_forceIntegralError += (forceError - 0.2 * m_forceIntegralError) * m_dt;
  std::cout << "################### task contact force m_forceIntegralError ok ###################" << std::endl;

  return m_constraint;
}

const ConstraintBase & TaskContactForceEquality::getConstraint() const {
  return m_constraint;
}

}
}

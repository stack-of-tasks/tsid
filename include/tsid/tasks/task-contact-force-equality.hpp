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

#ifndef __invdyn_task_contact_force_equality_hpp__
#define __invdyn_task_contact_force_equality_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-contact-force.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/contacts/contact-base.hpp"

namespace tsid {
namespace tasks {

class TaskContactForceEquality : public TaskContactForce {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef trajectories::TrajectorySample TrajectorySample;
  typedef math::Vector Vector;
  typedef math::Vector6 Vector6;
  typedef math::Vector3 Vector3;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef pinocchio::SE3 SE3;

  TaskContactForceEquality(const std::string& name, RobotWrapper& robot,
                           const double dt, contacts::ContactBase& contact);

  int dim() const override;

  virtual const std::string& getAssociatedContactName() override;
  virtual const contacts::ContactBase& getAssociatedContact();
  void setAssociatedContact(contacts::ContactBase& contact);

  // Task expressed as a PID between the reference force and the external one
  const ConstraintBase& compute(double t, ConstRefVector q, ConstRefVector v,
                                Data& data) override;

  const ConstraintBase& compute(
      double t, ConstRefVector q, ConstRefVector v, Data& data,
      const std::vector<std::shared_ptr<ContactLevel> >* contacts) override;

  const ConstraintBase& getConstraint() const override;

  void setReference(TrajectorySample& ref);
  const TrajectorySample& getReference() const;

  void setExternalForce(TrajectorySample& f_ext);
  const TrajectorySample& getExternalForce() const;

  const Vector& Kp() const;
  const Vector& Kd() const;
  const Vector& Ki() const;
  const double& getLeakRate() const;
  void Kp(ConstRefVector Kp);
  void Kd(ConstRefVector Kp);
  void Ki(ConstRefVector Ki);
  void setLeakRate(double leak);

 protected:
  // contact associated to the force task
  contacts::ContactBase* m_contact;
  std::string m_contact_name;  // the associated contact name or an empty string
  ConstraintEquality m_constraint;
  TrajectorySample m_ref;   // reference Force 6D to follow
  TrajectorySample m_fext;  // external Force 6D in the same frame than the ref
  Vector m_forceIntegralError;  // Integral error of the PID
  Vector m_Kp;
  Vector m_Kd;
  Vector m_Ki;
  double m_dt;
  double m_leak_rate;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_contact_force_equality_hpp__

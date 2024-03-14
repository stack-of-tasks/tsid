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

#ifndef __invdyn_task_joint_posture_hpp__
#define __invdyn_task_joint_posture_hpp__

#include <tsid/tasks/task-motion.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/math/constraint-equality.hpp>
#include <tsid/deprecated.hh>

namespace tsid {
namespace tasks {

class TaskJointPosture : public TaskMotion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef trajectories::TrajectorySample TrajectorySample;
  typedef math::Vector Vector;
  typedef math::VectorXi VectorXi;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef pinocchio::Data Data;

  TaskJointPosture(const std::string& name, RobotWrapper& robot);

  int dim() const override;

  const ConstraintBase& compute(const double t, ConstRefVector q,
                                ConstRefVector v, Data& data) override;

  const ConstraintBase& getConstraint() const override;

  void setReference(const TrajectorySample& ref);
  const TrajectorySample& getReference() const override;

  const Vector& getDesiredAcceleration() const override;
  Vector getAcceleration(ConstRefVector dv) const override;

  TSID_DEPRECATED const Vector& mask() const;     // deprecated
  TSID_DEPRECATED void mask(const Vector& mask);  // deprecated
  void setMask(math::ConstRefVector mask) override;

  const Vector& position_error() const override;
  const Vector& velocity_error() const override;
  const Vector& position() const override;
  const Vector& velocity() const override;
  const Vector& position_ref() const override;
  const Vector& velocity_ref() const override;

  const Vector& Kp();
  const Vector& Kd();
  void Kp(ConstRefVector Kp);
  void Kd(ConstRefVector Kp);

 protected:
  Vector m_Kp;
  Vector m_Kd;
  Vector m_p_error, m_v_error;
  Vector m_p, m_v;
  Vector m_a_des;
  VectorXi m_activeAxes;
  TrajectorySample m_ref;
  Vector m_ref_q_augmented;
  ConstraintEquality m_constraint;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_joint_posture_hpp__

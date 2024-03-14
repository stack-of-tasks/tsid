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

#ifndef __invdyn_task_am_equality_hpp__
#define __invdyn_task_am_equality_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid {
namespace tasks {

class TaskAMEquality : public TaskMotion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef trajectories::TrajectorySample TrajectorySample;
  typedef math::Vector Vector;
  typedef math::Vector3 Vector3;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef pinocchio::Data::Matrix6x Matrix6x;

  TaskAMEquality(const std::string& name, RobotWrapper& robot);

  int dim() const override;

  const ConstraintBase& compute(double t, ConstRefVector q, ConstRefVector v,
                                Data& data) override;

  const ConstraintBase& getConstraint() const override;

  void setReference(const TrajectorySample& ref);
  const TrajectorySample& getReference() const override;

  const Vector3& getDesiredMomentumDerivative() const;
  Vector3 getdMomentum(ConstRefVector dv) const;

  const Vector3& momentum_error() const;
  const Vector3& momentum() const;
  const Vector& momentum_ref() const;
  const Vector& dmomentum_ref() const;

  const Vector3& Kp();
  const Vector3& Kd();
  void Kp(ConstRefVector Kp);
  void Kd(ConstRefVector Kp);

 protected:
  Vector3 m_Kp;
  Vector3 m_Kd;
  Vector3 m_L_error, m_dL_error;
  Vector3 m_dL_des;

  Vector3 m_drift;
  Vector3 m_L, m_dL;
  TrajectorySample m_ref;
  ConstraintEquality m_constraint;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_am_equality_hpp__

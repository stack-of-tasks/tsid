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

#ifndef __invdyn_task_joint_bounds_hpp__
#define __invdyn_task_joint_bounds_hpp__

#include <tsid/tasks/task-motion.hpp>
#include <tsid/math/constraint-bound.hpp>

namespace tsid {
namespace tasks {

class TaskJointBounds : public TaskMotion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Vector Vector;
  typedef math::ConstraintBound ConstraintBound;
  typedef pinocchio::Data Data;

  TaskJointBounds(const std::string& name, RobotWrapper& robot, double dt);

  int dim() const override;

  const ConstraintBase& compute(const double t, ConstRefVector q,
                                ConstRefVector v, Data& data) override;

  const ConstraintBase& getConstraint() const override;

  void setTimeStep(double dt);
  void setVelocityBounds(ConstRefVector lower, ConstRefVector upper);
  void setAccelerationBounds(ConstRefVector lower, ConstRefVector upper);
  const Vector& getAccelerationLowerBounds() const;
  const Vector& getAccelerationUpperBounds() const;
  const Vector& getVelocityLowerBounds() const;
  const Vector& getVelocityUpperBounds() const;

  virtual void setMask(math::ConstRefVector mask) override;

 protected:
  Vector m_v_lb, m_v_ub;
  Vector m_a_lb, m_a_ub;
  Vector m_ddq_max_due_to_vel, m_ddq_min_due_to_vel;
  ConstraintBound m_constraint;
  double m_dt;
  int m_nv, m_na;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_joint_bounds_hpp__

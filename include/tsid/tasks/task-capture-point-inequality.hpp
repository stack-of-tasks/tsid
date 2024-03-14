//
// Copyright (c) 2020 CNRS, NYU, MPI Tübingen, PAL Robotics
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

#ifndef __invdyn_task_capture_point_inequality_hpp__
#define __invdyn_task_capture_point_inequality_hpp__

#include <tsid/tasks/task-motion.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/math/constraint-inequality.hpp>
#include <vector>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid {
namespace tasks {

class TaskCapturePointInequality : public TaskMotion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef math::Vector Vector;
  typedef math::Matrix Matrix;
  typedef math::Vector3 Vector3;
  typedef math::ConstraintInequality ConstraintInequality;
  typedef pinocchio::Data Data;
  typedef pinocchio::SE3 SE3;

  TaskCapturePointInequality(const std::string& name, RobotWrapper& robot,
                             const double timeStep);

  int dim() const override;

  const ConstraintBase& compute(double t, ConstRefVector q, ConstRefVector v,
                                Data& data) override;

  const ConstraintBase& getConstraint() const override;

  Vector getAcceleration(ConstRefVector dv) const override;

  const Vector& position() const override;

  void setSupportLimitsXAxis(const double x_min, const double x_max);

  void setSupportLimitsYAxis(const double y_min, const double y_max);

  void setSafetyMargin(const double x_margin, const double y_margin);

 protected:
  Vector m_drift_vec;
  Vector3 m_drift;
  Vector m_p_com, m_v_com;
  Vector m_rp_min;
  Vector m_rp_max;

  ConstraintInequality m_constraint;

  Vector m_safety_margin;
  Vector m_support_limits_x;
  Vector m_support_limits_y;

  Eigen::Index m_nv;
  double m_delta_t;
  double m_g;
  double m_w;
  double m_ka;
  int m_dim;

  Vector b_lower;
  Vector b_upper;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_capture_point_inequality_hpp__

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

#ifndef __invdyn_task_frames_equality_hpp__
#define __invdyn_task_frames_equality_hpp__

#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid {
namespace tasks {

class TaskTwoFramesEquality : public TaskMotion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef trajectories::TrajectorySample TrajectorySample;
  typedef math::Vector Vector;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef pinocchio::Data Data;
  typedef pinocchio::Data::Matrix6x Matrix6x;
  typedef pinocchio::Motion Motion;
  typedef pinocchio::SE3 SE3;

  TaskTwoFramesEquality(const std::string& name, RobotWrapper& robot,
                        const std::string& frameName1,
                        const std::string& frameName2);

  int dim() const;

  const ConstraintBase& compute(const double t, ConstRefVector q,
                                ConstRefVector v, Data& data);

  const ConstraintBase& getConstraint() const;

  /** Return the desired task acceleration (after applying the specified mask).
   *  The value is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  const Vector& getDesiredAcceleration() const;

  /** Return the task acceleration (after applying the specified mask).
   *  The value is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  Vector getAcceleration(ConstRefVector dv) const;

  virtual void setMask(math::ConstRefVector mask);

  /** Return the position tracking error (after applying the specified mask).
   *  The error is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  const Vector& position_error() const;

  /** Return the velocity tracking error (after applying the specified mask).
   *  The error is expressed in local frame is the local_frame flag is true,
   *  otherwise it is expressed in a local world-oriented frame.
   */
  const Vector& velocity_error() const;

  const Vector& Kp() const;
  const Vector& Kd() const;
  void Kp(ConstRefVector Kp);
  void Kd(ConstRefVector Kp);

  Index frame_id1() const;
  Index frame_id2() const;

  /**
   * @brief Specifies if the jacobian and desired acceloration should be
   * expressed in the local frame or the local world-oriented frame.
   *
   * @param local_frame If true, represent jacobian and acceloration in the
   *   local frame. If false, represent them in the local world-oriented frame.
   */
  void useLocalFrame(bool local_frame);

 protected:
  std::string m_frame_name1;
  std::string m_frame_name2;
  Index m_frame_id1;
  Index m_frame_id2;
  Motion m_p_error, m_v_error;
  Vector m_p_error_vec, m_v_error_vec;
  Vector m_p_error_masked_vec, m_v_error_masked_vec;
  Vector m_p, m_v;
  Vector m_p_ref, m_v_ref_vec;
  Motion m_v_ref, m_a_ref;
  SE3 m_wMl1, m_wMl2;
  Vector m_Kp;
  Vector m_Kd;
  Vector m_a_des, m_a_des_masked;
  Motion m_drift;
  Vector m_drift_masked;
  Matrix6x m_J1, m_J2;
  Matrix6x m_J1_rotated, m_J2_rotated;
  ConstraintEquality m_constraint;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_frames_equality_hpp__

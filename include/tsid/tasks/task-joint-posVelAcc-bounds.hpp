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

#ifndef __invdyn_task_joint_posVelAcc_bounds_hpp__
#define __invdyn_task_joint_posVelAcc_bounds_hpp__

#include <tsid/tasks/task-motion.hpp>
#include <tsid/math/constraint-bound.hpp>
#include <tsid/math/constraint-inequality.hpp>
#include <tsid/deprecated.hh>

/** This class has been implemented following :
 * Andrea del Prete. Joint Position and Velocity Bounds in Discrete-Time
 * Acceleration/Torque Control of Robot Manipulators. IEEE Robotics and
 * Automation Letters, IEEE 2018, 3 (1),
 * pp.281-288.￿10.1109/LRA.2017.2738321￿. hal-01356989v3 And
 * https://github.com/andreadelprete/pinocchio_inv_dyn/blob/master/python/pinocchio_inv_dyn/acc_bounds_util.py
 */
namespace tsid {
namespace tasks {

class TaskJointPosVelAccBounds : public TaskMotion {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Vector Vector;
  typedef math::ConstraintBound ConstraintBound;
  typedef math::ConstraintInequality ConstraintInequality;
  typedef math::VectorXi VectorXi;
  typedef pinocchio::Data Data;

  TaskJointPosVelAccBounds(const std::string& name, RobotWrapper& robot,
                           double dt, bool verbose = true);

  int dim() const override;

  const ConstraintBase& compute(double t, ConstRefVector q, ConstRefVector v,
                                Data& data) override;

  const ConstraintBase& getConstraint() const override;

  void setTimeStep(double dt);
  void setPositionBounds(ConstRefVector lower, ConstRefVector upper);
  void setVelocityBounds(ConstRefVector upper);
  void setAccelerationBounds(ConstRefVector upper);
  const Vector& getAccelerationBounds() const;
  const Vector& getVelocityBounds() const;
  const Vector& getPositionLowerBounds() const;
  const Vector& getPositionUpperBounds() const;

  void setVerbose(bool verbose);

  void setImposeBounds(bool impose_position_bounds, bool impose_velocity_bounds,
                       bool impose_viability_bounds,
                       bool impose_acceleration_bounds);

  /** Check if the state is viable, otherwise it returns a measure of the
   * violation of the violated inequality. Fills in m_viabViol , if the
   * state of joint i is viable m_viabViol[i] = 0
   */
  void isStateViable(ConstRefVector q, ConstRefVector dq, bool verbose = true);

  /** Compute acceleration limits imposed by position bounds.
   * Fills in  m_ddqLBPos and m_ddqUBPos
   */
  void computeAccLimitsFromPosLimits(ConstRefVector q, ConstRefVector dq,
                                     bool verbose = true);

  /** Compute acceleration limits imposed by viability.
   * ddqMax is the maximum acceleration that will be necessary to stop the
   * joint before hitting the position limits.
   *
   * -sqrt( 2*ddqMax*(q-qMin) ) < dq[t+1] < sqrt( 2*ddqMax*(qMax-q) )
   * ddqMin[2] = (-sqrt(max(0.0, 2*MAX_ACC*(q[i]+DT*dq[i]-qMin))) - dq[i])/DT;
   * ddqMax[2] = (sqrt(max(0.0, 2*MAX_ACC*(qMax-q[i]-DT*dq[i]))) - dq[i])/DT;
   *
   * Fills in  m_ddqLBVia and m_ddqUBVia
   */
  void computeAccLimitsFromViability(ConstRefVector q, ConstRefVector dq,
                                     bool verbose = true);

  /** Given the current position and velocity, the bounds of position,
   * velocity and acceleration and the control time step, compute the
   * bounds of the acceleration such that all the bounds are respected
   * at the next time step and can be respected in the future.
   * ddqMax is the absolute maximum acceleration.
   */
  void computeAccLimits(ConstRefVector q, ConstRefVector dq,
                        bool verbose = true);

  TSID_DEPRECATED const Vector& mask() const;     // deprecated
  TSID_DEPRECATED void mask(const Vector& mask);  // deprecated
  virtual void setMask(math::ConstRefVector mask) override;

 protected:
  ConstraintInequality m_constraint;
  double m_dt;
  bool m_verbose;
  int m_nv, m_na;

  Vector m_mask;
  VectorXi m_activeAxes;

  Vector m_qa;   // actuated part of q
  Vector m_dqa;  // actuated part of dq

  double m_eps;  // tolerance used to check violations

  Vector m_qMin;    // joints position limits
  Vector m_qMax;    // joints position limits
  Vector m_dqMax;   // joints max velocity limits
  Vector m_ddqMax;  // joints max acceleration limits

  Vector m_dqMinViab;  // velocity lower limits from viability
  Vector m_dqMaxViab;  // velocity upper limits from viability

  Vector m_ddqLBPos;  // acceleration lower bound from position bounds
  Vector m_ddqUBPos;  // acceleration upper bound from position bounds
  Vector m_ddqLBVia;  // acceleration lower bound from viability bounds
  Vector m_ddqUBVia;  // acceleration upper bound from viability bounds
  Vector m_ddqLBVel;  // acceleration lower bound from velocity bounds
  Vector m_ddqUBVel;  // acceleration upper bound from velocity bounds
  Vector m_ddqLBAcc;  // acceleration lower bound from acceleration bounds
  Vector m_ddqUBAcc;  // acceleration upper bound from acceleration bounds

  Vector m_ddqLB;  // final acceleration bounds
  Vector m_ddqUB;  // final acceleration bounds

  bool m_impose_position_bounds;
  bool m_impose_velocity_bounds;
  bool m_impose_viability_bounds;
  bool m_impose_acceleration_bounds;

  Vector m_viabViol;  // 0 if the state is viable, error otherwise

  // Used in computeAccLimitsFromPosLimits
  double m_two_dt_sq;
  Vector m_ddqMax_q3;
  Vector m_ddqMin_q3;
  Vector m_ddqMax_q2;
  Vector m_ddqMin_q2;
  Vector m_minus_dq_over_dt;

  // Used in computeAccLimitsFromViability
  double m_dt_square;
  Vector m_dt_dq;
  Vector m_dt_two_dq;
  Vector m_two_ddqMax;
  Vector m_dt_ddqMax_dt;
  Vector m_dq_square;
  Vector m_q_plus_dt_dq;
  double m_two_a;
  Vector m_b_1;
  Vector m_b_2;
  Vector m_ddq_1;
  Vector m_ddq_2;
  Vector m_c_1;
  Vector m_delta_1;
  Vector m_c_2;
  Vector m_delta_2;

  // Used in computeAccLimits
  Vector m_ub;
  Vector m_lb;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_joint_bounds_hpp__

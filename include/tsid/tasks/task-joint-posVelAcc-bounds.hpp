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

namespace tsid
{
  namespace tasks
  {

    class TaskJointPosVelAccBounds : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Vector Vector;
      typedef math::ConstraintBound ConstraintBound;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::VectorXi VectorXi;
      typedef pinocchio::Data Data;

      TaskJointPosVelAccBounds(const std::string & name,
                      RobotWrapper & robot,
                      double dt);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     const Data & data);

      const ConstraintBase & getConstraint() const;

      void setTimeStep(double dt);
      void setPositionBounds(ConstRefVector lower, ConstRefVector upper);
      void setVelocityBounds(ConstRefVector upper);
      void setAccelerationBounds(ConstRefVector upper);
      const Vector & getAccelerationBounds() const;
      const Vector & getVelocityBounds() const;
      const Vector & getPositionLowerBounds() const;
      const Vector & getPositionUpperBounds() const;


      void setImposeBounds(bool impose_position_bounds,
                           bool impose_velocity_bounds,
                           bool impose_viability_bounds,
                           bool impose_acceleration_bounds);

      void isStateViable(const Vector& q,const Vector& dq ,bool verbose=true);
      void computeVelLimits(const Vector& q, bool verbose=false);// velocity Limits From viability
      void computeAccLimitsFromPosLimits(const Vector&q,const Vector& dq, bool verbose=true);
      void computeAccLimitsFromViability(const Vector& q,const Vector& dq, bool verbose=true);
      void computeAccLimits(const Vector& q,const Vector& dq,bool verbose=true);
      void resetVectors();

      const Vector & mask() const;
      void mask(const Vector & mask);

    protected:
      ConstraintInequality m_constraint;
      double m_dt;
      int m_nv, m_na;

      Vector m_mask;
      VectorXi m_activeAxes;

      double eps; // tolerance used to check violations


      Vector m_qMin;//joints position limits
      Vector m_qMax;//joints position limits
      Vector m_dqMax;//joints max velocity limits
      Vector m_ddqMax;//joints max acceleration limits

      Vector m_dqMinViab;//velocity lower limits from viability
      Vector m_dqMaxViab;//velocity upper limits from viability

      Vector m_ddqLBPos;//acceleration lower bound from position bounds
      Vector m_ddqUBPos;//acceleration upper bound from position bounds
      Vector m_ddqLBVia;//acceleration lower bound from viability bounds
      Vector m_ddqUBVia;//acceleration upper bound from viability bounds
      Vector m_ddqLBVel;//acceleration lower bound from velocity bounds
      Vector m_ddqUBVel;//acceleration upper bound from velocity bounds
      Vector m_ddqLBAcc;//acceleration lower bound from acceleration bounds
      Vector m_ddqUBAcc;//acceleration upper bound from acceleration bounds

      Vector m_ddqLB;//final acceleration bounds
      Vector m_ddqUB;//final acceleration bounds

      bool m_impose_position_bounds;
      bool m_impose_velocity_bounds;
      bool m_impose_viability_bounds;
      bool m_impose_acceleration_bounds;
      bool m_verbose;


      Vector m_viabViol;

    };

  }
}

#endif // ifndef __invdyn_task_joint_bounds_hpp__

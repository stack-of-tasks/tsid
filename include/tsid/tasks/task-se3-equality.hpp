//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
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

#ifndef __invdyn_task_se3_equality_hpp__
#define __invdyn_task_se3_equality_hpp__

#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid
{
  namespace tasks
  {

    class TaskSE3Equality : public TaskMotion
    {
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

      TaskSE3Equality(const std::string & name,
                      RobotWrapper & robot,
                      const std::string & frameName);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     const Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;

      virtual void setMask(math::ConstRefVector mask);

      const Vector & position_error() const;
      const Vector & velocity_error() const;
      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector & Kp() const;
      const Vector & Kd() const;
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

      Index frame_id() const;

      /**
       * @brief Specifies if the jacobian and desired acceloration should be
       * expressed in the local frame or the local world oriented frame.
       *
       * @param local_frame If true, represent jacobian and acceloration in the
       *   local frame. If false, represent them in the local world oriented frame.
       */
      void useLocalFrame(bool local_frame);

    protected:

      std::string m_frame_name;
      Index m_frame_id;
      Motion m_p_error, m_v_error;
      Vector m_p_error_vec, m_v_error_vec;
      Vector m_p, m_v;
      Vector m_p_ref, m_v_ref_vec;
      Motion m_v_ref, m_a_ref;
      SE3 m_M_ref, m_wMl;
      Vector m_Kp;
      Vector m_Kd;
      Vector m_a_des;
      Motion m_drift;
      Matrix6x m_J;
      Matrix6x m_J_rotated;
      ConstraintEquality m_constraint;
      TrajectorySample m_ref;
      bool m_local_frame;
    };

  }
}

#endif // ifndef __invdyn_task_se3_equality_hpp__

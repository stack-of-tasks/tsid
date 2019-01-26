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

#include "tsid/tasks/task-com-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskComEquality::TaskComEquality(const std::string & name,
                                     RobotWrapper & robot):
      TaskMotion(name, robot),
      m_constraint(name, 3, robot.nv())
    {
      m_Kp.setZero(3);
      m_Kd.setZero(3);
      m_p_error_vec.setZero(3);
      m_v_error_vec.setZero(3);
      m_p_com.setZero(3);
      m_v_com.setZero(3);
      m_a_des_vec.setZero(3);
    }

    int TaskComEquality::dim() const
    {
      //return self._mask.sum ()
      return 3;
    }

    const Vector3 & TaskComEquality::Kp(){ return m_Kp; }

    const Vector3 & TaskComEquality::Kd(){ return m_Kd; }

    void TaskComEquality::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==3);
      m_Kp = Kp;
    }

    void TaskComEquality::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==3);
      m_Kd = Kd;
    }

    void TaskComEquality::setReference(const TrajectorySample & ref)
    {
      m_ref = ref;
    }

    const TrajectorySample & TaskComEquality::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskComEquality::getDesiredAcceleration() const
    {
      return m_a_des_vec;
    }

    Vector TaskComEquality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv - m_drift;
    }

    const Vector & TaskComEquality::position_error() const
    {
      return m_p_error_vec;
    }

    const Vector & TaskComEquality::velocity_error() const
    {
      return m_v_error_vec;
    }

    const Vector & TaskComEquality::position() const
    {
      return m_p_com;
    }

    const Vector & TaskComEquality::velocity() const
    {
      return m_v_com;
    }

    const Vector & TaskComEquality::position_ref() const
    {
      return m_ref.pos;
    }

    const Vector & TaskComEquality::velocity_ref() const
    {
      return m_ref.vel;
    }

    const ConstraintBase & TaskComEquality::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskComEquality::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    const Data & data)
    {
      m_robot.com(data, m_p_com, m_v_com, m_drift);

      // Compute errors
      m_p_error = m_p_com - m_ref.pos;
      m_v_error = m_v_com - m_ref.vel;
      m_a_des = - m_Kp.cwiseProduct(m_p_error)
                - m_Kd.cwiseProduct(m_v_error)
                + m_ref.acc;

      m_p_error_vec = m_p_error;
      m_v_error_vec = m_v_error;
      m_a_des_vec = m_a_des;
#ifndef NDEBUG
//      std::cout<<m_name<<" errors: "<<m_p_error.norm()<<" "
//        <<m_v_error.norm()<<std::endl;
#endif

      // Get CoM jacobian
      const Matrix3x & Jcom = m_robot.Jcom(data);

      m_constraint.setMatrix(Jcom);
      m_constraint.setVector(m_a_des - m_drift);
      return m_constraint;
    }
    
  }
}

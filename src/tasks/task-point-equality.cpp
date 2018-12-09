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

#include "tsid/math/utils.hpp"
#include "tsid/tasks/task-point-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace se3;

    TaskPointEquality::TaskPointEquality(const std::string & name,
                                     RobotWrapper & robot,
                                     const std::string & frameName):
      TaskMotion(name, robot),
      m_frame_name(frameName),
      m_constraint(name, 3, robot.nv()),
      m_ref(12, 6)
    {
      assert(m_robot.model().existFrame(frameName));
      m_frame_id = m_robot.model().getFrameId(frameName);

      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_p_error_vec.setZero(3);
      m_v_error_vec.setZero(3);
      m_p.resize(12);
      m_v.resize(6);
      m_p_ref.resize(12);
      m_v_ref_vec.resize(6);
      m_Kp.setZero(3);
      m_Kd.setZero(3);
      m_a_des.setZero(3);
      m_J.setZero(6, robot.nv());
    }

    int TaskPointEquality::dim() const
    {
      //return self._mask.sum ()
      return 6;
    }

    const Vector & TaskPointEquality::Kp() const { return m_Kp; }

    const Vector & TaskPointEquality::Kd() const { return m_Kd; }

    void TaskPointEquality::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==3);
      m_Kp = Kp;
    }

    void TaskPointEquality::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==3);
      m_Kd = Kd;
    }

    void TaskPointEquality::setReference(TrajectorySample & ref)
    {
      m_ref = ref;
      vectorToSE3(ref.pos, m_M_ref);
      m_v_ref = Motion(ref.vel);
      m_a_ref = Motion(ref.acc);
    }

    const TrajectorySample & TaskPointEquality::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskPointEquality::position_error() const
    {
      return m_p_error_vec;
    }

    const Vector & TaskPointEquality::velocity_error() const
    {
      return m_v_error_vec;
    }

    const Vector & TaskPointEquality::position() const
    {
      return m_p;
    }

    const Vector & TaskPointEquality::velocity() const
    {
      return m_v;
    }

    const Vector & TaskPointEquality::position_ref() const
    {
      return m_p_ref;
    }

    const Vector & TaskPointEquality::velocity_ref() const
    {
      return m_v_ref_vec;
    }

    const Vector & TaskPointEquality::getDesiredAcceleration() const
    {
      return m_a_des;
    }

    Vector TaskPointEquality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv + m_drift.toVector();
    }

    Index TaskPointEquality::frame_id() const
    {
      return m_frame_id;
    }

    const ConstraintBase & TaskPointEquality::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskPointEquality::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector ,
                                                    const Data & data)
    {
      SE3 oMi;
      Motion v_frame;
      m_robot.framePosition(data, m_frame_id, oMi);
      m_robot.frameVelocity(data, m_frame_id, v_frame);
      m_robot.frameClassicAcceleration(data, m_frame_id, m_drift);

      // Transformation from local to world
      m_wMl.rotation(oMi.rotation());

      m_v_error = m_wMl.act(v_frame) - m_v_ref;  // vel err in world frame

      SE3ToVector(m_M_ref, m_p_ref);
      m_v_ref_vec = m_v_ref.toVector();

      m_p_error_vec = oMi.translation() - m_p_ref.topRows(3);
      m_v_error_vec = m_v_error.toVector().topRows(3);

      SE3ToVector(oMi, m_p);
      m_v = v_frame.toVector();

#ifndef NDEBUG
//      PRINT_VECTOR(v_frame.toVector());
//      PRINT_VECTOR(m_v_ref.toVector());
#endif

      // desired acc in world frame
      m_a_des = - m_Kp.cwiseProduct(m_p_error_vec)
                - m_Kd.cwiseProduct(m_v_error_vec)
                + m_a_ref.linear();

      // Express the jacobian and desired acceloration in world oriented frame.
      m_robot.frameJacobianLocalWorldOriented(data, m_frame_id, m_J);

      m_constraint.setMatrix(m_J.topRows(3));
      m_constraint.setVector(m_a_des - oMi.act(m_drift).linear());
      return m_constraint;
    }
    
  }
}

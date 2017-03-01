//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#include <pininvdyn/tasks/task-se3-equality.hpp>

namespace pininvdyn
{
  namespace tasks
  {
    using namespace pininvdyn::math;
    using namespace se3;

    TaskSE3Equality::TaskSE3Equality(const std::string & name,
                                     RobotWrapper & robot,
                                     const std::string & frameName):
      TaskMotion(name, robot),
      m_frame_name(frameName),
      m_constraint(name, 6, robot.nv())
    {
      assert(m_robot.model().existFrame(frameName));
      m_frame_id = m_robot.model().getFrameId(frameName);

      m_v_ref.setZero();
      m_a_ref.setZero();
      m_M_ref.setIdentity();
      m_wMl.setIdentity();
      m_Kp.setZero(6);
      m_Kd.setZero(6);
      m_a_des.setZero(6);
      m_J.setZero(6, robot.nv());
    }

    int TaskSE3Equality::dim() const
    {
      //return self._mask.sum ()
      return 6;
    }

    const Vector & TaskSE3Equality::Kp(){ return m_Kp; }

    const Vector & TaskSE3Equality::Kd(){ return m_Kd; }

    void TaskSE3Equality::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==6);
      m_Kp = Kp;
    }

    void TaskSE3Equality::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==6);
      m_Kd = Kd;
    }

    void TaskSE3Equality::setReference(TrajectorySample & ref)
    {
      vectorToSE3(ref.pos, m_M_ref);
      m_v_ref = Motion(ref.vel);
      m_a_ref = Motion(ref.acc);
    }

    const ConstraintBase & TaskSE3Equality::compute(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      SE3 oMi;
      Motion v_frame, drift;
      m_robot.framePosition(data, m_frame_id, oMi);
      m_robot.frameVelocity(data, m_frame_id, v_frame);
      m_robot.frameClassicAcceleration(data, m_frame_id, drift);

      // Transformation from local to world
      m_wMl.rotation(oMi.rotation());

      errorInSE3(oMi, m_M_ref, m_p_error);          // pos err in local frame
      m_v_error = v_frame - m_wMl.actInv(m_v_ref);  // vel err in local frame

      // desired acc in local frame
      m_a_des = - m_Kp.cwiseProduct(m_p_error.toVector_impl())
                - m_Kd.cwiseProduct(m_v_error.toVector_impl())
                + m_wMl.actInv(m_a_ref).toVector_impl();

      m_robot.frameJacobianLocal(data, m_frame_id, m_J);

//      if(local_frame==False):
//        drift = self._gMl.act(drift);
//      a_des[:3] = self._gMl.rotation * a_des[:3];
//      a_des[3:] = self._gMl.rotation * a_des[3:];
//      J[:3,:] = self._gMl.rotation * J[:3,:];
//      J[3:,:] = self._gMl.rotation * J[3:,:];

      m_constraint.setMatrix(m_J);
      m_constraint.setVector(m_a_des - drift.toVector_impl());
      return m_constraint;
    }
    
  }
}

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

#include <tsid/tasks/task-joint-posture.hpp>
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/math/utils.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskJointPosture::TaskJointPosture(const std::string & name,
                                     RobotWrapper & robot):
      TaskMotion(name, robot),
      m_ref(robot.na()),
      m_constraint(name, robot.na(), robot.nv())
    {
      m_Kp.setZero(robot.na());
      m_Kd.setZero(robot.na());
      Vector m = Vector::Ones(robot.na());
      setMask(m);
    }

    const Vector & TaskJointPosture::mask() const
    {
      return m_mask;
    }

    void TaskJointPosture::mask(const Vector & m)
    {
      // std::cerr<<"The method TaskJointPosture::mask is deprecated. Use TaskJointPosture::setMask instead.\n";
      return setMask(m);
    }

    void TaskJointPosture::setMask(ConstRefVector m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
      const Vector::Index dim = static_cast<Vector::Index>(m.sum());
      Matrix S = Matrix::Zero(dim, m_robot.nv());
      m_activeAxes.resize(dim);
      unsigned int j=0;
      for(unsigned int i=0; i<m.size(); i++)
        if(m(i)!=0.0)
        {
          assert(m(i)==1.0);
          S(j,m_robot.nv()-m_robot.na()+i) = 1.0;
          m_activeAxes(j) = i;
          j++;
        }
      m_constraint.resize((unsigned int)dim, m_robot.nv());
      m_constraint.setMatrix(S);
      m_J = S;
    }

    int TaskJointPosture::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskJointPosture::Kp(){ return m_Kp; }

    const Vector & TaskJointPosture::Kd(){ return m_Kd; }

    void TaskJointPosture::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==m_robot.na());
      m_Kp = Kp;
    }

    void TaskJointPosture::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==m_robot.na());
      m_Kd = Kd;
    }

    void TaskJointPosture::setReference(const TrajectorySample & ref)
    {
      assert(ref.pos.size()==m_robot.na());
      assert(ref.vel.size()==m_robot.na());
      assert(ref.acc.size()==m_robot.na());
      m_ref = ref;
    }

    const TrajectorySample & TaskJointPosture::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskJointPosture::getDesiredAcceleration() const
    {
      return m_a_des;
    }

    Vector TaskJointPosture::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv;
    }

    const Vector & TaskJointPosture::position_error() const
    {
      return m_p_error;
    }

    const Vector & TaskJointPosture::velocity_error() const
    {
      return m_v_error;
    }

    const Vector & TaskJointPosture::position() const
    {
      return m_p;
    }

    const Vector & TaskJointPosture::velocity() const
    {
      return m_v;
    }

    const Vector & TaskJointPosture::position_ref() const
    {
      return m_ref.pos;
    }

    const Vector & TaskJointPosture::velocity_ref() const
    {
      return m_ref.vel;
    }

    const Vector & TaskJointPosture::acceleration_ref() const
    {
      return m_ref.acc;
    }
    
    const ConstraintBase & TaskJointPosture::getConstraint() const
    {
      return m_constraint;
    }
    const Matrix & TaskJointPosture::getJacobian() const
    { 
      return m_J; 
    }

    const ConstraintBase & TaskJointPosture::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      // Compute errors
      m_p = q.tail(m_robot.na());
      m_v = v.tail(m_robot.na());
      m_p_error = m_p - m_ref.pos;
      m_v_error = m_v - m_ref.vel;
      // Matrix Lambda_inv, Lambda, Jpinv;
      // Jpinv.setZero(m_J.cols(), m_J.rows());
      // pseudoInverse(m_J, Jpinv, 1e-6);
      // Lambda = Jpinv.transpose() * m_robot.mass(data) * Jpinv;
      // Lambda_inv.setZero(Lambda.cols(), Lambda.rows());
      // pseudoInverse(Lambda, Lambda_inv, 1e-6);
      // Matrix LKP = (Lambda_inv * m_Kp).cwiseAbs();
      // Matrix LKD = (Lambda_inv * m_Kd).cwiseAbs();
      //std::cout << "##################### Lambda_inv posture: "<<  Lambda_inv << "################################" << std::endl;
      // std::cout << "##################### Lambda_inv * m_Kp posture: "<<  LKP << "################################" << std::endl;
      // std::cout << "##################### Lambda_inv * m_Kd posture: "<<  LKD << "################################" << std::endl;
      m_a_des = - m_Kp.cwiseProduct(m_p_error) - m_Kd.cwiseProduct(m_v_error) + m_ref.acc; //- Lambda_inv * m_Kp.cwiseProduct(m_p_error) - Lambda_inv * m_Kd.cwiseProduct(m_v_error)
                

      for(unsigned int i=0; i<m_activeAxes.size(); i++)
        m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
      return m_constraint;
    }
    
  }
}

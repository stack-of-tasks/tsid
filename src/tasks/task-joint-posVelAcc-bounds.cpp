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

#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include "tsid/robots/robot-wrapper.hpp"

/** This class has been implemented following :
* Andrea del Prete. Joint Position and Velocity Bounds in Discrete-Time 
* Acceleration/Torque Control of Robot Manipulators. IEEE Robotics and Automation
* Letters, IEEE 2018, 3 (1), pp.281-288.￿10.1109/LRA.2017.2738321￿. hal-01356989v3
* And
* https://github.com/andreadelprete/pinocchio_inv_dyn/blob/master/python/pinocchio_inv_dyn/acc_bounds_util.py
*/
namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskJointPosVelAccBounds::TaskJointPosVelAccBounds(const std::string & name,
                                     RobotWrapper & robot,
                                     double dt):
      TaskMotion(name, robot),
      m_constraint(name, robot.na(), robot.nv()),
      m_dt(2*dt),
      m_nv(robot.nv()),
      m_na(robot.na())
    {
      assert(dt>0.0);
      eps = 1e-10;
      resetVectors();
      m_qMin=Vector::Constant(m_na,1,1e10);
      m_qMax=Vector::Constant(m_na,1,-1e10);
      m_dqMax=Vector::Constant(m_na,1,1e10);
      m_ddqMax=Vector::Constant(m_na,1,1e10);
      m_impose_position_bounds=false;
      m_impose_velocity_bounds=false;
      m_impose_viability_bounds = false;
      m_impose_acceleration_bounds = false;
      m_verbose = true;

      //Used in computeAccLimitsFromPosLimits
      m_two_dt_sq = 2.0/(m_dt*m_dt);
      m_ddqMax_q3 = Vector::Zero(m_na);
      m_ddqMin_q3 = Vector::Zero(m_na);
      m_ddqMax_q2 = Vector::Zero(m_na);
      m_ddqMin_q2 = Vector::Zero(m_na);
      m_minus_dq_over_dt = Vector::Zero(m_na);


      //Used in computeAccLimitsFromViability
      m_dt_square = m_dt*m_dt;
      m_two_a = 2*m_dt_square;
      m_dt_dq = Vector::Zero(m_na);
      m_dt_two_dq  = Vector::Zero(m_na);
      m_two_ddqMax = Vector::Zero(m_na);
      m_dt_ddqMax_dt = Vector::Zero(m_na);
      m_dq_square = 0.0;
      m_q_plus_dt_dq = Vector::Zero(m_na);
      m_b_1 = Vector::Zero(m_na);
      m_b_2 = Vector::Zero(m_na);
      m_ddq_1 = Vector::Zero(m_na);
      m_ddq_2 = Vector::Zero(m_na);
      m_c_1 = Vector::Zero(m_na);
      m_delta_1 = Vector::Zero(m_na);
      m_c_2 = Vector::Zero(m_na);
      m_delta_2 = Vector::Zero(m_na);

      //Used in computeAccLimits
      m_ub = Vector::Constant(4,1,1e10);
      m_lb =  Vector::Constant(4,1,-1e10);

      Vector m = Vector::Ones(robot.na());
      mask(m);

      for(int i=0; i<m_na; i++)
      {
        m_constraint.upperBound()(i) = 1e10;
        m_constraint.lowerBound()(i) = -1e10;
      }
    }

    const Vector & TaskJointPosVelAccBounds::mask() const
    {
      return m_mask;
    }

    void TaskJointPosVelAccBounds::mask(const Vector & m)
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
    }

    int TaskJointPosVelAccBounds::dim() const
    { return m_na; }

    const Vector & TaskJointPosVelAccBounds::getAccelerationBounds() const
    { return m_ddqMax; }

    const Vector & TaskJointPosVelAccBounds::getVelocityBounds() const
    { return m_dqMax; }

    const Vector & TaskJointPosVelAccBounds::getPositionLowerBounds() const
    { return m_qMin; }

    const Vector & TaskJointPosVelAccBounds::getPositionUpperBounds() const
    { return m_qMax; }

    void TaskJointPosVelAccBounds::setTimeStep(double dt)
    {
      assert(dt>0);
      m_dt = dt;
    }

    void TaskJointPosVelAccBounds::setPositionBounds(ConstRefVector lower, ConstRefVector upper)
    {
      assert(lower.size()==m_na);
      assert(upper.size()==m_na);
      m_qMin = lower;
      m_qMax = upper;
      m_impose_position_bounds=true;
      m_impose_viability_bounds=true;
    }

    void TaskJointPosVelAccBounds::setVelocityBounds(ConstRefVector upper)
    {
      assert(upper.size()==m_na);
      m_dqMax = upper;
      m_impose_velocity_bounds = true;
    }

    void TaskJointPosVelAccBounds::setAccelerationBounds(ConstRefVector upper)
    {
      assert(upper.size()==m_na);
      m_ddqMax = upper;
      m_impose_acceleration_bounds = true;
    }

    const ConstraintBase & TaskJointPosVelAccBounds::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskJointPosVelAccBounds::compute(const double ,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    const Data & )
    {
      computeAccLimits(q,v,m_verbose);
      m_constraint.upperBound()= m_ddqUB;
      m_constraint.lowerBound()= m_ddqLB;
      resetVectors();
      return m_constraint;
    }

    void TaskJointPosVelAccBounds::setImposeBounds(bool impose_position_bounds,
                                                   bool impose_velocity_bounds,
                                                   bool impose_viability_bounds,
                                                   bool impose_acceleration_bounds)
    {
      m_impose_position_bounds=impose_position_bounds;
      m_impose_velocity_bounds=impose_velocity_bounds;
      m_impose_viability_bounds=impose_viability_bounds;
      m_impose_acceleration_bounds=impose_acceleration_bounds;
    }


    void TaskJointPosVelAccBounds::isStateViable(const Vector& qa,
                                                            const Vector& dqa ,
                                                            bool verbose)
    {
      for(int i = 0; i<m_na; i++)
      {
        if(qa[i] < (m_qMin[i] - eps))
        {
          if(verbose)
          {
            std::cout << "State of joint "<< i <<" is not viable because q[i]< qMin[i] : "
            <<qa[i] <<"<"<< m_qMin[i] <<std::endl;
          }
          m_viabViol[i] = m_qMin[i] - qa[i];
        }
        if(qa[i] > (m_qMax[i] + eps))
        {
          if(verbose){
            std::cout << "State of joint "<< i <<" is not viable because qa[i]>m_qMax[i] : "
            << qa[i] <<">"<< m_qMax[i] <<std::endl;
          }
          m_viabViol[i] =qa[i]-m_qMax[i];
        }
        if(std::abs(dqa[i]) > (m_dqMax[i] + eps))
        {
          if(verbose)
          {
            std::cout << "State (q,dq) :("<<qa[i]<<","<<dqa[i]<<") of joint "
            << i <<" is not viable because |dq|>dqMax : "<< std::abs(dqa[i]) <<
            ">"<< m_dqMax[i]<<std::endl;
          }
          m_viabViol[i] =std::abs(dqa[i])-m_dqMax[i];
        }
        double dqMaxViab =   std::sqrt(std::max(0.0, 2*m_ddqMax[i]*(m_qMax[i]-qa[i])));
        if(dqa[i]>(dqMaxViab+eps))
        {
          if(verbose)
          {
            std::cout << "State (q,dq,dqMaxViab) :("<<qa[i]<<","<<dqa[i]<<","<<
            dqMaxViab<<") of joint "<< i <<" is not viable because dq>dqMaxViab : "
            << dqa[i] <<">"<< dqMaxViab<<std::endl;
          }
          m_viabViol[i] =dqa[i]-dqMaxViab;
        }
        double dqMinViab =  -std::sqrt(std::max(0.0,2*m_ddqMax[i]*(qa[i]-m_qMin[i])));
        if(dqa[i]<(dqMinViab+eps))
        {
          if(verbose)
          {
            std::cout << "State (q,dq,dqMinViab) :("<<qa[i]<<","<<dqa[i]<<","<<
            dqMinViab<<") of joint "<< i <<" is not viable because dq<dqMinViab : "
            << dqa[i] <<"<"<< dqMinViab<<std::endl;
          }
          m_viabViol[i]=dqMinViab-dqa[i];
        }
      }
    }

    void TaskJointPosVelAccBounds::computeAccLimitsFromPosLimits(const Vector&qa,
                                                                  const Vector& dqa,
                                                                  bool verbose)
    {
      m_ddqMax_q3 = m_two_dt_sq*(m_qMax-qa-m_dt*dqa);
      m_ddqMin_q3 = m_two_dt_sq*(m_qMin-qa-m_dt*dqa);
      m_ddqMax_q2 = Vector::Zero(m_na);
      m_ddqMin_q2 = Vector::Zero(m_na);
      m_minus_dq_over_dt = -dqa/m_dt;
      for(int i = 0; i < m_na; i ++)
      {
        if(dqa[i]<=0.0)
        {
          m_ddqUBPos[i]  = m_ddqMax_q3[i];
          if(m_ddqMin_q3[i] < m_minus_dq_over_dt[i])
          {
            m_ddqLBPos[i]  = m_ddqMin_q3[i];
          }
          else if(qa[i]!=m_qMin[i])
          {
            m_ddqMin_q2[i] = (dqa[i]*dqa[i])/(2.0*(qa[i]-m_qMin[i]));
            m_ddqLBPos[i]  = std::max(m_ddqMin_q2[i],m_minus_dq_over_dt[i]);
          }
          else
          {
            if(verbose == true)
            {
              std::cout << "WARNING  qa[i]==m_qMin[i] for joint" << i << std::endl;
              std::cout << "You are goting to violate the position bound " << i << std::endl;
            }
            m_ddqLBPos[i] = 0.0;
          }
        }
        else
        {
          m_ddqLBPos[i]  = m_ddqMin_q3[i];
          if(m_ddqMax_q3[i] > m_minus_dq_over_dt[i])
          {
            m_ddqUBPos[i]  = m_ddqMax_q3[i];
          }
          else if(qa[i]!=m_qMax[i])
          {
            m_ddqMax_q2[i] = -(dqa[i]*dqa[i])/(2*(m_qMax[i]-qa[i]));
            m_ddqUBPos[i]  = std::min(m_ddqMax_q2[i],m_minus_dq_over_dt[i]);
          }
          else
          {
            if(verbose == true)
            {
              std::cout << "WARNING  qa[i]==m_qMax[i] for joint" << i << std::endl;
              std::cout << "You are goting to violate the position bound " << i << std::endl;
            }
            m_ddqUBPos[i] = 0.0;
          }
        }
      }
    }
    void TaskJointPosVelAccBounds::computeAccLimitsFromViability(const Vector& qa,
                                                                const Vector& dqa,
                                                                bool verbose)
    {
      m_dt_dq = m_dt*dqa;
      m_minus_dq_over_dt = -dqa/m_dt;
      m_dt_two_dq = 2*m_dt_dq;
      m_two_ddqMax = 2*m_ddqMax;
      m_dt_ddqMax_dt = m_ddqMax*m_dt_square;
      m_dq_square = dqa.dot(dqa);
      m_q_plus_dt_dq = qa + m_dt_dq;
      m_b_1 = m_dt_two_dq + m_dt_ddqMax_dt;
      m_b_2 = m_dt_two_dq - m_dt_ddqMax_dt;
      m_ddq_1 = Vector::Zero(m_na);
      m_ddq_2 = Vector::Zero(m_na);
      m_c_1 = m_dq_square - m_two_ddqMax.cwiseProduct(m_qMax - m_q_plus_dt_dq).array();
      m_delta_1 = m_b_1.cwiseProduct(m_b_1) - 2*m_two_a*m_c_1;
      m_c_2 = m_dq_square - m_two_ddqMax.cwiseProduct(m_q_plus_dt_dq - m_qMin).array();
      m_delta_2 = m_b_2.cwiseProduct(m_b_2) - 2*m_two_a*m_c_2;
      for(int i=0; i<m_na; i++)
      {
        if(m_delta_1[i]>=0.0)
        {
          m_ddq_1[i] = (-m_b_1[i] + std::sqrt(m_delta_1[i]))/(m_two_a);
        }
        else{
          m_ddq_1[i] = m_minus_dq_over_dt[i];
          if(verbose==true)
          {
            std::cout << "Error: state (" << qa[i] <<"," << dqa[i] <<") of joint "<< 
            i <<  "not viable because delta is negative: "<< m_delta_1 << std::endl;
          }
        }
        if(m_delta_2[i] >= 0.0)
        {
          m_ddq_2[i] = (-m_b_2[i] - std::sqrt(m_delta_2[i]))/(m_two_a);
        }
        else
        {
          m_ddq_2[i] = m_minus_dq_over_dt[i];
          if(verbose==true)
          {
            std::cout << "Error: state (" << qa[i] <<"," << dqa[i] <<") of joint "<< 
              i <<  "not viable because delta is negative: "<< m_delta_2 << std::endl;
          }
        }
      }
      m_ddqUBVia = m_ddq_1.cwiseMax(m_minus_dq_over_dt);
      m_ddqLBVia = m_ddq_2.cwiseMin(m_minus_dq_over_dt);
    }

    void TaskJointPosVelAccBounds::computeAccLimits(const Vector& q,const Vector& dq, bool verbose)
    {
      isStateViable(q.tail(m_na), dq.tail(m_na), m_verbose);
      if(verbose==true)
      {
        for(int i = 0; i<m_na; i++)
        {
          if(m_viabViol[i]>eps)
          {
            std::cout << "WARNING: specified state ( < " <<q.tail(m_na)[i]<< " , " << dq.tail(m_na)[i]
            <<") is not viable violation : "<< m_viabViol[i] << std::endl;
          }
        }
      }

      //Acceleration limits imposed by position bounds
      if(m_impose_position_bounds==true)
      {
          computeAccLimitsFromPosLimits(q.tail(m_na), dq.tail(m_na), verbose);
      }
      // Acceleration limits imposed by velocity bounds
      // dq[t+1] = dq + dt*ddq < dqMax
      // ddqMax = (dqMax-dq)/dt
      // ddqMin = (dqMin-dq)/dt = (-dqMax-dq)/dt
      if(m_impose_velocity_bounds==true)
      {
        m_ddqLBVel=(-m_dqMax-dq.tail(m_na))/m_dt;
        m_ddqUBVel= (m_dqMax-dq.tail(m_na))/m_dt;
      }
      //Acceleration limits imposed by viability
      if(m_impose_viability_bounds==true)
      {
        computeAccLimitsFromViability(q.tail(m_na), dq.tail(m_na), verbose);
      }
      //Acceleration limits
      if(m_impose_acceleration_bounds==true)
      {
        m_ddqLBAcc = -m_ddqMax;
        m_ddqUBAcc = m_ddqMax;
      }
      //Take the most conservative limit for each joint
      m_ub = Vector::Constant(4,1,1e10);
      m_lb =  Vector::Constant(4,1,-1e10);

      for(int i = 0; i<m_na; i++)
      {
        m_ub[0] = m_ddqUBPos[i];
        m_ub[1] = m_ddqUBVia[i];
        m_ub[2] = m_ddqUBVel[i];
        m_ub[3] = m_ddqUBAcc[i];

        m_lb[0] = m_ddqLBPos[i];
        m_lb[1] = m_ddqLBVia[i];
        m_lb[2] = m_ddqLBVel[i];
        m_lb[3] = m_ddqLBAcc[i];

        m_ddqLB[i]=m_lb.maxCoeff();
        m_ddqUB[i]=m_ub.minCoeff();
        
        if(m_ddqUB[i] < m_ddqLB[i])
        {
          if(verbose==true)
          {
            std::cout << "Conflict between pos/vel/acc bound ddqMin " <<m_ddqLB[i]
            << " ddqMax " << m_ddqUB[i] << std::endl;
            std::cout << "ub " << m_ub.transpose() << std::endl;
            std::cout << "lb " << m_lb.transpose() << std::endl;
          }
          if(m_ddqUB[i] == m_ub[0])
          {
            m_ddqLB[i] = m_ddqUB[i];
          }
          else
          {
            m_ddqUB[i] = m_ddqLB[i];
          }
          if(verbose==true)
          {
            std::cout << "New bounds are  ddqMin " <<m_ddqLB[i]<< " ddqMax "
            << m_ddqUB[i] << std::endl;
          }
        }
      }

    }

    void TaskJointPosVelAccBounds::resetVectors()
    {
      m_ddqLBPos=Vector::Constant(m_na,1,-1e10);
      m_ddqUBPos=Vector::Constant(m_na,1,1e10);
      m_ddqLBVia=Vector::Constant(m_na,1,-1e10);
      m_ddqUBVia=Vector::Constant(m_na,1,1e10);
      m_ddqLBVel=Vector::Constant(m_na,1,-1e10);
      m_ddqUBVel=Vector::Constant(m_na,1,1e10);
      m_ddqLBAcc=Vector::Constant(m_na,1,-1e10);
      m_ddqUBAcc=Vector::Constant(m_na,1,1e10);
      m_ddqLB=Vector::Constant(m_na,1,-1e10);
      m_ddqUB=Vector::Constant(m_na,1,1e10);
      m_viabViol = Vector::Zero(m_na);
    }
  }
}

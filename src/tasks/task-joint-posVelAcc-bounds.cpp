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

      Vector m = Vector::Ones(robot.na());
      mask(m);

      int offset = m_nv-m_na;
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
      // std::cout << S << std::endl;
      // std::cout << "rows dim " << dim <<std::endl;
      // std::cout << "cols dim " <<  m_robot.nv() <<std::endl;
      // std::cout << m_constraint.vector() << std::endl;
      // std::cout << m_constraint.vector().size() << std::endl;
      // std::cout << m_constraint.matrix() << std::endl;

    }

    int TaskJointPosVelAccBounds::dim() const
    { return m_nv; }

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

      for(int i=0; i<m_na; i++)
      {
        m_constraint.upperBound()(i) = m_ddqUB[i];
        m_constraint.lowerBound()(i) = m_ddqLB[i];
      }
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


     void TaskJointPosVelAccBounds::isStateViable(const Vector& q,
                                                            const Vector& dq ,
                                                            bool verbose)
    {
      Vector qa = q.tail(m_na);
      Vector dqa = dq.tail(m_na);


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

      // if((m_viabViol == Vector::Zero(m_na))&&verbose==true)
      // {
      //   // std::cout << "State (q,dq) :("<<qa.transpose()<<","<<dqa.transpose()<<"is viable"<<std::endl;
      //   std::cout << "State is viable"<<std::endl;
      // }
    }

    void TaskJointPosVelAccBounds::computeVelLimits(const Vector& q, bool verbose)
     {
        Vector qa = q.tail(m_na);
        for(int i = 0; i<m_na; i++)
        {
        if(verbose)
        {
          if(qa[i] < (m_qMin[i] - eps))
          {
            std::cerr << "State of joint "<< i <<" is not viable because q[i]<qMin[i] : "
            << qa[i] <<"<"<< m_qMin[i] <<std::endl;
          }
          if(qa[i] < (m_qMax[i] + eps))
          {
            std::cerr << "State of joint "<< i <<" is not viable because qa[i]>m_qMax[i] : "
            << qa[i] <<">"<< m_qMax[i] <<std::endl;
          }
          m_dqMaxViab[i] = std::min( m_dqMax[i],  std::sqrt(std::max(0.0,2*m_ddqMax[i]*(m_qMax[i]-qa[i]))));
          m_dqMinViab[i] = std::max(-m_dqMax[i], -std::sqrt(std::max(0.0,2*m_ddqMax[i]*(qa[i]-m_qMin[i]))));
        }
    }
  }


    void TaskJointPosVelAccBounds::computeAccLimitsFromPosLimits(const Vector&q,
                                                                const Vector& dq,
                                                                bool verbose)
  {
        Vector qa = q.tail(m_na);
        Vector dqa = dq.tail(m_na);
        double two_dt_sq   = 2.0/(std::pow(m_dt,2));
        Vector ddqMax_q3 = two_dt_sq*(m_qMax-qa-m_dt*dqa);
        Vector ddqMin_q3 = two_dt_sq*(m_qMin-qa-m_dt*dqa);
        Vector ddqMax_q2 = Vector::Zero(m_na);
        Vector ddqMin_q2 = Vector::Zero(m_na);
        Vector minus_dq_over_dt = -dqa/m_dt;
        for(int i = 0; i < m_na; i ++)
        {
          if(dqa[i]<=0.0)
          {
              m_ddqUBPos[i]  = ddqMax_q3[i];
              if(ddqMin_q3[i] < minus_dq_over_dt[i])
              {
                  m_ddqLBPos[i]  = ddqMin_q3[i];
              }
              else if(qa[i]!=m_qMin[i])
              {
                  ddqMin_q2[i] = std::pow(dqa[i],2)/(2.0*(qa[i]-m_qMin[i]));
                  m_ddqLBPos[i]  = std::max(ddqMin_q2[i],minus_dq_over_dt[i]);
              }
              else
              {
                  if(verbose == true){
                    std::cout << "WARNING  qa[i]==m_qMin[i] for joint" << i << std::endl;
                    std::cout << "You are goting to violate the position bound " << i << std::endl;
                  }
                  m_ddqLBPos[i] = 0.0;
              }
          }
          else
          {
              m_ddqLBPos[i]  = ddqMin_q3[i];
              if(ddqMax_q3[i] > minus_dq_over_dt[i])
              {
                  m_ddqUBPos[i]  = ddqMax_q3[i];
              }
              else if(qa[i]!=m_qMax[i])
              {
                  ddqMax_q2[i] = -std::pow(dqa[i],2)/(2*(m_qMax[i]-qa[i]));
                  m_ddqUBPos[i]  = std::min(ddqMax_q2[i],minus_dq_over_dt[i]);
              }
              else
              {
                  if(verbose == true){
                    std::cout << "WARNING  qa[i]==m_qMax[i] for joint" << i << std::endl;
                    std::cout << "You are goting to violate the position bound " << i << std::endl;
                  }
                  m_ddqUBPos[i] = 0.0;
              }
          }
       }
    }
    void TaskJointPosVelAccBounds::computeAccLimitsFromViability(const Vector& q,
                                                                const Vector& dq,
                                                                bool verbose)
    {
        Vector qa = q.tail(m_na);
        Vector dqa = dq.tail(m_na);
        double dt_square = std::pow(m_dt,2);
        Vector dt_dq = m_dt*dqa;
        Vector minus_dq_over_dt = -dqa/m_dt;
        Vector dt_two_dq = 2*dt_dq;
        Vector two_ddqMax = 2*m_ddqMax;
        Vector dt_ddqMax_dt = m_ddqMax*dt_square;
        double dq_square = dqa.dot(dqa);
        Vector q_plus_dt_dq = qa + dt_dq;
        double two_a = 2*dt_square;
        Vector b = dt_two_dq + dt_ddqMax_dt;
        Vector bl = dt_two_dq - dt_ddqMax_dt;
        for(int i=0; i<m_na; i++)
        {
          double ddq_1 = 0;
          double ddq_2 = 0;
          double c = dq_square - two_ddqMax[i]*(m_qMax[i] - q_plus_dt_dq[i]);
          double delta = std::pow(b[i],2) - 2*two_a*c;
          if(delta>=0.0)
          {
              ddq_1 = (-b[i] + std::sqrt(delta))/(two_a);
          }
          else{
              ddq_1 = minus_dq_over_dt[i];
              if(verbose==true){
                  std::cout << "Error: state (" << qa[i] <<"," << dqa[i] <<") of joint "<< 
                  i <<  "not viable because delta is negative: "<< delta << std::endl;
              }
          }
          c = dq_square - two_ddqMax[i]*(q_plus_dt_dq[i] - m_qMin[i]);
          delta = std::pow(bl[i],2) - 2*two_a*c;
          if(delta >= 0.0)
          {
              ddq_2 = (-bl[i] - std::sqrt(delta))/(two_a);
          }
          else
          {
              ddq_2 = minus_dq_over_dt[i];
              if(verbose==true)
              {
                std::cout << "Error: state (" << qa[i] <<"," << dqa[i] <<") of joint "<< 
                  i <<  "not viable because delta is negative: "<< delta << std::endl;
              }
          }
          m_ddqUBVia[i] = std::max(ddq_1, minus_dq_over_dt[i]);
          m_ddqLBVia[i] = std::min(ddq_2, minus_dq_over_dt[i]);
        }
    }
    void TaskJointPosVelAccBounds::computeAccLimits(const Vector& q,const Vector& dq, bool verbose)
    {
      Vector qa = q.tail(m_na);
      Vector dqa = dq.tail(m_na);
      isStateViable(q, dq, m_verbose);

      if(verbose==true)
      {
        for(int i = 0; i<m_na; i++)
        {
          if(m_viabViol[i]>eps)
          {
            std::cout << "WARNING: specified state ( < " <<qa[i]<< " , " << dqa[i]
            <<") is not viable violation : "<< m_viabViol[i] << std::endl;
          }
        }
      }

      //Acceleration limits imposed by position bounds
      if(m_impose_position_bounds==true)
      {
          computeAccLimitsFromPosLimits(q, dq, verbose);
      }
      // Acceleration limits imposed by velocity bounds
      // dq[t+1] = dq + dt*ddq < dqMax
      // ddqMax = (dqMax-dq)/dt
      // ddqMin = (dqMin-dq)/dt = (-dqMax-dq)/dt
      if(m_impose_velocity_bounds==true)
      {
          m_ddqLBVel=(-m_dqMax-dqa)/m_dt;
          m_ddqUBVel= (m_dqMax-dqa)/m_dt;
      }
      //Acceleration limits imposed by viability
      if(m_impose_viability_bounds==true)
      {
          computeAccLimitsFromViability(q, dq, verbose);
      }
      //Acceleration limits
      if(m_impose_acceleration_bounds==true)
      {
          m_ddqLBAcc = -m_ddqMax;
          m_ddqUBAcc = m_ddqMax;
      }
      //Take the most conservative limit for each joint
      Vector ub = Vector::Constant(4,1,1e10);
      Vector lb =  Vector::Constant(4,1,-1e10);

      for(int i = 0; i<m_na; i++)
      {
        ub[0] = m_ddqUBPos[i];
        ub[1] = m_ddqUBVia[i];
        ub[2] = m_ddqUBVel[i];
        ub[3] = m_ddqUBAcc[i];

        lb[0] = m_ddqLBPos[i];
        lb[1] = m_ddqLBVia[i];
        lb[2] = m_ddqLBVel[i];
        lb[3] = m_ddqLBAcc[i];

        m_ddqLB[i]=lb.maxCoeff();
        m_ddqUB[i]=ub.minCoeff();
        
        if(m_ddqUB[i] < m_ddqLB[i])
        {
          if(verbose==true)
          {
            std::cout << "Conflict between pos/vel/acc bound ddqMin " <<m_ddqLB[i]
            << " ddqMax " << m_ddqUB[i] << std::endl;
            std::cout << "ub " << ub.transpose() << std::endl;
            std::cout << "lb " << lb.transpose() << std::endl;

          }
          if(m_ddqUB[i] == ub[0])
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

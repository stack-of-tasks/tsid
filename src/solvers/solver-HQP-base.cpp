//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//

#include "pininvdyn/solvers/solver-HQP-base.h"


namespace pininvdyn
{
  namespace solvers
  {

    std::string const Solver_HQP_base::HQP_status_string[] = { "HQP_STATUS_OPTIMAL",
                                                  "HQP_STATUS_INFEASIBLE",
                                                  "HQP_STATUS_UNBOUNDED",
                                                  "HQP_STATUS_MAX_ITER_REACHED",
                                                  "HQP_STATUS_ERROR"};

    std::string hqpDataToString(const HqpData & data, bool printMatrices)
    {
      stringstream ss;
      unsigned int priority = 0;
      for(HqpData::const_iterator it=data.begin(); it!=data.end(); it++)
      {
        ss<<"Level "<< priority<<endl;
        for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
        {
          const pininvdyn::math::ConstraintBase* c = iit->second;
          ss<<" - "<<c->name()<<": w="<<iit->first<<", ";
          if(c->isEquality())
            ss<<"equality, ";
          else if(c->isInequality())
            ss<<"inequality, ";
          else
            ss<<"bound, ";
          ss<<c->rows()<<"x"<<c->cols()<<endl;
        }
        priority++;
      }

      if(printMatrices)
      {
        ss<<endl;
        for(HqpData::const_iterator it=data.begin(); it!=data.end(); it++)
        {
          for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
          {
            const pininvdyn::math::ConstraintBase* c = iit->second;
            ss<<"*** "<<c->name()<<" *** ";
            if(c->isEquality())
            {
              ss<<"(equality)"<<endl;
              ss<<"A =\n"<<c->matrix()<<endl;
              ss<<"b = "<<c->vector().transpose()<<endl;
            }
            else if(c->isInequality())
            {
              ss<<"(inequality)"<<endl;
              ss<<"A =\n"<<c->matrix()<<endl;
              ss<<"lb = "<<c->lowerBound().transpose()<<endl;
              ss<<"ub = "<<c->upperBound().transpose()<<endl;
            }
            else
            {
              ss<<"(bounds)"<<endl;
              ss<<"lb = "<<c->lowerBound().transpose()<<endl;
              ss<<"ub = "<<c->upperBound().transpose()<<endl;
            }
            ss<<endl;
          }
        }
      }
      return ss.str();
    }
          
    Solver_HQP_base::Solver_HQP_base(const std::string & name)
    {
      m_name = name;
      m_maxIter = 1000;
      m_maxTime = 100.0;
      m_useWarmStart = true;
    }

    bool Solver_HQP_base::setMaximumIterations(unsigned int maxIter)
    {
      if(maxIter==0)
        return false;
      m_maxIter = maxIter;
      return true;
    }

    bool Solver_HQP_base::setMaximumTime(double seconds)
    {
      if(seconds<=0.0)
        return false;
      m_maxTime = seconds;
      return true;
    }
    
  }
}

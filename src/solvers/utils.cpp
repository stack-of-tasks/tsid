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

#include "tsid/solvers/utils.hpp"
#include "tsid/math/constraint-base.hpp"

#include <iostream>

namespace tsid
{
  namespace solvers
  {
    
    std::string HQPDataToString(const HQPData & data, bool printMatrices)
    {
      using namespace std;
      
      stringstream ss;
      unsigned int priority = 0;
      for(HQPData::const_iterator it=data.begin(); it!=data.end(); it++)
      {
        ss<<"Level "<< priority<<endl;
        for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
        {
          const math::ConstraintBase* c = iit->second;
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
        for(HQPData::const_iterator it=data.begin(); it!=data.end(); it++)
        {
          for(ConstraintLevel::const_iterator iit=it->begin(); iit!=it->end(); iit++)
          {
            const math::ConstraintBase* c = iit->second;
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
    
  }
}

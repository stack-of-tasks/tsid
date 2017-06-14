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

#include "tsid/solvers/solver-HQP-base.hpp"

#include <iostream>

namespace tsid
{
  namespace solvers
  {

    std::string const SolverHQPBase::HQP_status_string[] = { "HQP_STATUS_OPTIMAL",
                                                  "HQP_STATUS_INFEASIBLE",
                                                  "HQP_STATUS_UNBOUNDED",
                                                  "HQP_STATUS_MAX_ITER_REACHED",
                                                  "HQP_STATUS_ERROR"};

    SolverHQPBase::SolverHQPBase(const std::string & name)
    {
      m_name = name;
      m_maxIter = 1000;
      m_maxTime = 100.0;
      m_useWarmStart = true;
    }

    bool SolverHQPBase::setMaximumIterations(unsigned int maxIter)
    {
      if(maxIter==0)
        return false;
      m_maxIter = maxIter;
      return true;
    }

    bool SolverHQPBase::setMaximumTime(double seconds)
    {
      if(seconds<=0.0)
        return false;
      m_maxTime = seconds;
      return true;
    }
    
  }
}

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

#ifndef __invdyn_solvers_hqp_output_hpp__
#define __invdyn_solvers_hqp_output_hpp__

#include "pininvdyn/solvers/fwd.hpp"
#include "pininvdyn/math/fwd.hpp"

#include <vector>


namespace pininvdyn
{
  namespace solvers
  {
    
    typedef std::vector< std::pair<double, math::ConstraintBase*> > ConstraintLevel;
    typedef std::vector< std::pair<double, const math::ConstraintBase*> > ConstConstraintLevel;
    typedef std::vector<ConstraintLevel> HQPData;
    typedef std::vector<ConstConstraintLevel> ConstHQPData;
    
    std::string HQPDataToString(const HQPData & data, bool printMatrices=false);
    
    class HQPOutput
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      HQPStatus status;                    /// solver status
      math::Vector x;            /// solution
      math::Vector lambda;       /// Lagrange multipliers
      math::VectorXi activeSet;  /// indexes of active inequalities
      int iterations;                       /// number of iterations performed by the solver
      
      HQPOutput(){}
      
      HQPOutput(int nVars, int nEqCon, int nInCon)
      {
        resize(nVars, nEqCon, nInCon);
      }
      
      void resize(int nVars, int nEqCon, int nInCon)
      {
        x.resize(nVars);
        lambda.resize(nEqCon+nInCon);
        activeSet.resize(nInCon);
      }
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_output_hpp__

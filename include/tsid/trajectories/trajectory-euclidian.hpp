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

#ifndef __invdyn_trajectory_euclidian_hpp__
#define __invdyn_trajectory_euclidian_hpp__

#include <tsid/trajectories/trajectory-base.hpp>

namespace tsid
{
  namespace trajectories
  {

    class TrajectoryEuclidianConstant : public TrajectoryBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Vector         Vector;
      typedef math::ConstRefVector ConstRefVector;

      TrajectoryEuclidianConstant(const std::string & name);

      TrajectoryEuclidianConstant(const std::string & name, ConstRefVector ref);

      unsigned int size() const;

      void setReference(ConstRefVector ref);

      const TrajectorySample & operator()(double time);

      const TrajectorySample & computeNext();

      void getLastSample(TrajectorySample & sample) const;

      bool has_trajectory_ended() const;

    protected:
      Vector m_ref;
    };
    
  }
}

#endif // ifndef __invdyn_trajectory_euclidian_hpp__

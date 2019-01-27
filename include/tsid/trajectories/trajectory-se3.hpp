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

#ifndef __invdyn_trajectory_se3_hpp__
#define __invdyn_trajectory_se3_hpp__

#include <tsid/trajectories/trajectory-base.hpp>

#include <pinocchio/spatial/se3.hpp>

namespace tsid
{
  namespace trajectories
  {

    class TrajectorySE3Constant : public TrajectoryBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef pinocchio::SE3 SE3;

      TrajectorySE3Constant(const std::string & name);

      TrajectorySE3Constant(const std::string & name, const SE3 & M);

      unsigned int size() const;

      void setReference(const SE3 & M);

      const TrajectorySample & operator()(double time);

      const TrajectorySample & computeNext();

      void getLastSample(TrajectorySample & sample) const;

      bool has_trajectory_ended() const;


    protected:
      SE3    m_M;
    };
    
  }
}

#endif // ifndef __invdyn_trajectory_se3_hpp__

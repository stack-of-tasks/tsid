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

#ifndef __invdyn_trajectory_base_hpp__
#define __invdyn_trajectory_base_hpp__

#include "tsid/math/fwd.hpp"

#include <string>

namespace tsid
{
  namespace trajectories
  {
    
    class TrajectorySample
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      math::Vector pos, vel, acc;

      TrajectorySample(unsigned int size=0)
      {
        resize(size, size);
      }

      TrajectorySample(unsigned int size_pos, unsigned int size_vel)
      {
        resize(size_pos, size_vel);
      }

      void resize(unsigned int size)
      {
        resize(size, size);
      }

      void resize(unsigned int size_pos, unsigned int size_vel)
      {
        pos.setZero(size_pos);
        vel.setZero(size_vel);
        acc.setZero(size_vel);
      }
    };


    class TrajectoryBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      TrajectoryBase(const std::string & name):
        m_name(name){}

      virtual unsigned int size() const = 0;

      virtual const TrajectorySample & operator()(double time) = 0;

      virtual const TrajectorySample & computeNext() = 0;

      virtual const TrajectorySample & getLastSample() const { return m_sample; }

      virtual void getLastSample(TrajectorySample & sample) const = 0;

      virtual bool has_trajectory_ended() const = 0;

    protected:
      std::string m_name;
      TrajectorySample m_sample;
    };
  }
}

#endif // ifndef __invdyn_trajectory_base_hpp__

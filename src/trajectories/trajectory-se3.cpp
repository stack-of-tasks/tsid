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

#include <pininvdyn/trajectories/trajectory-se3.hpp>

using namespace pininvdyn::math;

namespace pininvdyn
{
  namespace trajectories
  {

    TrajectorySE3Constant::TrajectorySE3Constant(const std::string & name)
      :TrajectoryBase(name)
    {}

    TrajectorySE3Constant::TrajectorySE3Constant(const std::string & name,
                                                 const SE3 & M)
      :TrajectoryBase(name)
    {
      m_sample.resize(12, 6);
      se3ToVector(M, m_sample.pos);
      m_sample.vel.setZero();
    }

    unsigned int TrajectorySE3Constant::size() const
    {
      return 6;
    }

    const TrajectorySample & TrajectorySE3Constant::operator()(double time)
    {
      return m_sample;
    }

    const TrajectorySample & TrajectorySE3Constant::computeNext()
    {
      return m_sample;
    }

    void TrajectorySE3Constant::getLastSample(TrajectorySample & sample) const
    {
      sample = m_sample;
    }

    bool TrajectorySE3Constant::has_trajectory_ended() const
    {
      return true;
    }

  }
}

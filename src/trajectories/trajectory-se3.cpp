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

#include "tsid/math/utils.hpp"
#include "tsid/trajectories/trajectory-se3.hpp"

using namespace tsid::math;

namespace tsid
{
  namespace trajectories
  {

    TrajectorySE3Constant::TrajectorySE3Constant(const std::string & name)
      :TrajectoryBase(name)
    {
      m_sample.resize(12, 6);
    }

    TrajectorySE3Constant::TrajectorySE3Constant(const std::string & name,
                                                 const SE3 & M)
      :TrajectoryBase(name)
    {
      m_sample.resize(12, 6);
      SE3ToVector(M, m_sample.pos);
    }

    unsigned int TrajectorySE3Constant::size() const
    {
      return 6;
    }

    void TrajectorySE3Constant::setReference(const pinocchio::SE3 & ref)
    {
      m_sample.resize(12, 6);
      SE3ToVector(ref, m_sample.pos);
    }

    const TrajectorySample & TrajectorySE3Constant::operator()(double )
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

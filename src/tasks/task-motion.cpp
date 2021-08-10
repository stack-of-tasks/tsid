//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
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

#include "tsid/tasks/task-motion.hpp"

namespace tsid
{
  namespace tasks
  {

    typedef math::Vector Vector;
    typedef math::Matrix Matrix;
    typedef trajectories::TrajectorySample TrajectorySample;

    TaskMotion::TaskMotion(const std::string & name,
                           RobotWrapper & robot):
      TaskBase(name, robot)
    {}
    
    void TaskMotion::setMask(math::ConstRefVector mask)
    {
      m_mask = mask;
    }

    bool TaskMotion::hasMask()
    {
      return m_mask.size() > 0;
    }
    const std::string TaskMotion::getFrameName() { return ""; }

    const Vector & TaskMotion::getMask() const { return m_mask; }
    const Vector & TaskMotion::Kp(){ return m_Kp_dummy; }
    const Vector & TaskMotion::Kd(){ return m_Kd_dummy; }

    void TaskMotion::Kp(ConstRefVector Kp){ m_Kp_dummy = Kp; }
    void TaskMotion::Kd(ConstRefVector Kd){ m_Kd_dummy = Kd; }

    const TrajectorySample & TaskMotion::getReference() const { return TrajectorySample_dummy; }

    const Matrix & TaskMotion::getJacobian() const { return m_Matrix_dummy; }

    const Vector & TaskMotion::getDesiredAcceleration() const  { return m_dummy; }

    Vector TaskMotion::getAcceleration(ConstRefVector ) const  { return m_dummy; }

    const Vector & TaskMotion::position_error() const { return m_dummy; }
    const Vector & TaskMotion::velocity_error() const  { return m_dummy; }
    const Vector & TaskMotion::position() const  { return m_dummy; }
    const Vector & TaskMotion::velocity() const  { return m_dummy; }
    const Vector & TaskMotion::position_ref() const  { return m_dummy; }
    const Vector & TaskMotion::velocity_ref() const  { return m_dummy; }
    const Vector & TaskMotion::acceleration_ref() const { return m_dummy; }

  }
}

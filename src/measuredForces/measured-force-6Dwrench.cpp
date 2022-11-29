//
// Copyright (c) 2022 CNRS INRIA LORIA
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

#include "tsid/measuredForces/measured-force-6Dwrench.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace measuredForces
  {

    using namespace std;
    using namespace math;
    using namespace pinocchio;

    MeasuredForce6Dwrench::MeasuredForce6Dwrench(const std::string & name,
                                                  RobotWrapper & robot,
                                                  const std::string & frameName):
      MeasuredForceBase(name, robot),
      m_frame_name(frameName)
    {
      assert(m_robot.model().existFrame(frameName));
      m_frame_id = m_robot.model().getFrameId(frameName);

      m_fext.setZero(6);
      m_J.setZero(6, robot.nv());
      m_computedTorques.setZero(robot.nv());

      m_local_frame = true;
    }


    const Vector & MeasuredForce6Dwrench::computeJointTorques(const double ,
                                            ConstRefVector ,
                                            ConstRefVector ,
                                            Data & data)
    {
        m_robot.frameJacobianLocal(data, m_frame_id, m_J);
        m_computedTorques = m_J.transpose() * m_fext;

        //@todo handle case where fext is given in the local world-oriented frame
        return m_computedTorques;
    }

    void MeasuredForce6Dwrench::setMeasuredContactForce(Vector & fext)
    {
        assert(fext.size()==6 ); //TODO assert not working
        m_fext = fext;
    }

    const Vector & MeasuredForce6Dwrench::getMeasuredContactForce() const
    {
        return m_fext;
    }

    void MeasuredForce6Dwrench::useLocalFrame(bool local_frame)
    {
      m_local_frame = local_frame;
    }

  }
}

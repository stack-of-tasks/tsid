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

#ifndef __invdyn_measured_force_3Dforce_hpp__
#define __invdyn_measured_force_3Dforce_hpp__

#include "tsid/measuredForces/measured-force-base.hpp"
#include <pinocchio/multibody/data.hpp>

namespace tsid
{
  namespace measuredForces
  {
    class MeasuredForce3Dforce : public MeasuredForceBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef math::Vector3 Vector3;
      typedef robots::RobotWrapper RobotWrapper;
      typedef pinocchio::Data Data;
      typedef pinocchio::Data::Matrix3x Matrix3x;

      MeasuredForce3Dforce(const std::string & name,
                        RobotWrapper & robot,
                        const std::string & frameName);


      const Vector & computeJointTorques(Data & data);

    /**
     *  Set the value of the external wrench applied by the environment on the robot.
     */
     void setMeasuredContactForce(Vector3 & fext);

     const Vector3 & getMeasuredContactForce() const;

     /**
      * @brief Specifies if the external force and jacobian is
      * expressed in the local frame or the local world-oriented frame.
      *
      * @param local_frame If true, represent external force and jacobian in the
      *   local frame. If false, represent them in the local world-oriented frame.
      */
     void useLocalFrame(bool local_frame);

     protected:
       std::string m_frame_name;
       Index m_frame_id;
       Vector3 m_fext;
       Matrix3x m_J;
       Matrix3x m_J_rotated;
       Vector m_computedTorques;
       bool m_local_frame;
    };
  }
}

#endif // ifndef __invdyn_measured_force_6Dwrench_hpp__

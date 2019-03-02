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

#ifndef __invdyn_contact_base_hpp__
#define __invdyn_contact_base_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/robots/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"


namespace tsid
{
  namespace contacts
  {
    
    ///
    /// \brief Base template of a Contact.
    ///
    class ContactBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::ConstraintBase ConstraintBase;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::Matrix Matrix;
      typedef tasks::TaskMotion TaskMotion;
      typedef pinocchio::Data Data;
      typedef robots::RobotWrapper RobotWrapper;

      ContactBase(const std::string & name,
                  RobotWrapper & robot);

      const std::string & name() const;

      void name(const std::string & name);
      
      /// Return the number of motion constraints
      virtual unsigned int n_motion() const = 0;

      /// Return the number of force variables
      virtual unsigned int n_force() const = 0;

      virtual const ConstraintBase & computeMotionTask(const double t,
                                                       ConstRefVector q,
                                                       ConstRefVector v,
                                                       const Data & data) = 0;

      virtual const ConstraintInequality & computeForceTask(const double t,
                                                            ConstRefVector q,
                                                            ConstRefVector v,
                                                            const Data & data) = 0;

      virtual const Matrix & getForceGeneratorMatrix() = 0;

      virtual const ConstraintEquality & computeForceRegularizationTask(const double t,
                                                                        ConstRefVector q,
                                                                        ConstRefVector v,
                                                                        const Data & data) = 0;

      virtual const TaskMotion & getMotionTask() const = 0;
      virtual const ConstraintBase & getMotionConstraint() const = 0;
      virtual const ConstraintInequality & getForceConstraint() const = 0;
      virtual const ConstraintEquality & getForceRegularizationTask() const = 0;
      
      virtual double getMinNormalForce() const = 0;
      virtual double getMaxNormalForce() const = 0;
      virtual bool setMinNormalForce(const double minNormalForce) = 0;
      virtual bool setMaxNormalForce(const double maxNormalForce) = 0;
      virtual double getNormalForce(ConstRefVector f) const = 0;

    protected:
      std::string m_name;
      /// \brief Reference on the robot model.
      RobotWrapper & m_robot;
    };
    
  }
}

#endif // ifndef __invdyn_contact_base_hpp__

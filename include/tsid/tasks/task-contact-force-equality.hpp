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

#ifndef __invdyn_task_contact_force_equality_hpp__
#define __invdyn_task_contact_force_equality_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-contact-force.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/formulations/inverse-dynamics-formulation-base.hpp"

namespace tsid
{
  namespace tasks
  {

    class TaskContactForceEquality : public TaskContactForce
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector6 Vector6;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::SE3 SE3;

      TaskContactForceEquality(const std::string & name,
                      		   RobotWrapper & robot,
                      		   const std::string & contactName="");

      void setContactList(const std::vector<std::shared_ptr<ContactLevel> >  *contacts);

      int dim() const;

      virtual const std::string& getAssociatedContactName();
      virtual void setAssociatedContactName(const std::string & contactName);

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data,
                                     const std::vector<std::shared_ptr<ContactLevel> >  *contacts);

      const ConstraintBase & getConstraint() const;

      void setReference(TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      void setExternalForce(TrajectorySample & f_ext);
      const TrajectorySample & getExternalForce() const;

      const Vector & Kp() const;
      const Vector & Kd() const;
      const Vector & Ki() const;
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);
      void Ki(ConstRefVector Ki);

    protected:
      const std::vector<std::shared_ptr<ContactLevel> > *m_contacts;
      std::string m_contact_name; // an empty string
      ConstraintEquality m_constraint;
      TrajectorySample m_ref;  // reference Force in world frame
      TrajectorySample m_fext;     // external force in the world frame
      Vector6 m_forceIntegralError;
	  Vector m_Kp;
      Vector m_Kd;
      Vector m_Ki;
      double m_dt;
    };

  }
}

#endif // ifndef __invdyn_task_contact_force_equality_hpp__

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

#ifndef __invdyn_contact_point_hpp__
#define __invdyn_contact_point_hpp__

#include "tsid/contacts/contact-base.hpp"
#include "tsid/tasks/task-se3-equality.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"

namespace tsid
{
  namespace contacts
  {
    class ContactPoint : public ContactBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::ConstRefMatrix ConstRefMatrix;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::Matrix3x Matrix3x;
      typedef math::Vector6 Vector6;
      typedef math::Vector3 Vector3;
      typedef math::Vector Vector;
      typedef tasks::TaskMotion TaskMotion;
      typedef tasks::TaskSE3Equality TaskSE3Equality;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::SE3 SE3;

      ContactPoint(const std::string & name,
                RobotWrapper & robot,
                const std::string & frameName,
                ConstRefVector contactNormal,
                const double frictionCoefficient,
                const double minNormalForce,
                const double maxNormalForce);

      /// Return the number of motion constraints
      virtual unsigned int n_motion() const;

      /// Return the number of force variables
      virtual unsigned int n_force() const;

      virtual const ConstraintBase & computeMotionTask(const double t,
                                                       ConstRefVector q,
                                                       ConstRefVector v,
                                                       const Data & data);

      virtual const ConstraintInequality & computeForceTask(const double t,
                                                            ConstRefVector q,
                                                            ConstRefVector v,
                                                            const Data & data);

      virtual const Matrix & getForceGeneratorMatrix();

      virtual const ConstraintEquality & computeForceRegularizationTask(const double t,
                                                                        ConstRefVector q,
                                                                        ConstRefVector v,
                                                                        const Data & data);

      const TaskMotion & getMotionTask() const;
      const ConstraintBase & getMotionConstraint() const;
      const ConstraintInequality & getForceConstraint() const;
      const ConstraintEquality & getForceRegularizationTask() const;
      double getMotionTaskWeight() const;

      double getNormalForce(ConstRefVector f) const;
      double getMinNormalForce() const;
      double getMaxNormalForce() const;

      const Vector & Kp() const;
      const Vector & Kd() const;
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

      bool setContactNormal(ConstRefVector contactNormal);

      bool setFrictionCoefficient(const double frictionCoefficient);
      bool setMinNormalForce(const double minNormalForce);
      bool setMaxNormalForce(const double maxNormalForce);
      bool setMotionTaskWeight(const double w);
      void setReference(const SE3 & ref);
      void setForceReference(ConstRefVector & f_ref);
      void setRegularizationTaskWeightVector(ConstRefVector & w);

      /**
       * @brief Specifies if properties of the contact point and motion task
       * are expressed in the local or local world oriented frame. The contact
       * forces, contact normal and contact coefficients are interpreted in
       * the specified frame.
       *
       * @param local_frame If true, use the local frame, otherwise use the
       * local world oriented
       */
      void useLocalFrame(bool local_frame);

    protected:

      void updateForceInequalityConstraints();
      void updateForceRegularizationTask();
      void updateForceGeneratorMatrix();

      TaskSE3Equality m_motionTask;
      ConstraintInequality m_forceInequality;
      ConstraintEquality m_forceRegTask;
      Vector3 m_contactNormal;
      Vector3 m_fRef;
      Vector3 m_weightForceRegTask;
      double m_mu;
      double m_fMin;
      double m_fMax;
      double m_regularizationTaskWeight;
      double m_motionTaskWeight;
      Matrix m_forceGenMat;
    };
  }
}

#endif // ifndef __invdyn_contact_6d_hpp__

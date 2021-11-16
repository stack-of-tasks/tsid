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

#ifndef __invdyn_task_actuation_equality_hpp__
#define __invdyn_task_actuation_equality_hpp__

#include <tsid/tasks/task-actuation.hpp>
#include <tsid/math/constraint-equality.hpp>

namespace tsid
{
  namespace tasks
  {
    class TaskActuationEquality : public TaskActuation
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef math::Vector Vector;
      typedef math::VectorXi VectorXi;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;

      TaskActuationEquality(const std::string & name,
                            RobotWrapper & robot);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(math::ConstRefVector ref); 
      const Vector & getReference() const;

      void setNormalizationVector(math::ConstRefVector norm);
      const Vector & getNormalizationVector() const;

      const Vector & mask() const;
      void mask(const Vector & mask);

    protected:
      Vector m_mask;
      VectorXi m_activeAxes;
      Vector m_ref;
      Vector m_norm;
      ConstraintEquality m_constraint;
    };
  }
}

#endif // ifndef __invdyn_task_actuation_equality_hpp__

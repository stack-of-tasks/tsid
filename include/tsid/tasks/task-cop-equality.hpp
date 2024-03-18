//
// Copyright (c) 2021 University of Trento
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

#ifndef __invdyn_task_cop_equality_hpp__
#define __invdyn_task_cop_equality_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-contact-force.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/formulations/inverse-dynamics-formulation-base.hpp"

namespace tsid {
namespace tasks {

class TaskCopEquality : public TaskContactForce {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef trajectories::TrajectorySample TrajectorySample;
  typedef math::Vector Vector;
  typedef math::Vector3 Vector3;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef pinocchio::SE3 SE3;

  TaskCopEquality(const std::string& name, RobotWrapper& robot);

  void setContactList(
      const std::vector<std::shared_ptr<ContactLevel> >* contacts);

  int dim() const override;

  const std::string& getAssociatedContactName() override;

  const ConstraintBase& compute(double t, ConstRefVector q, ConstRefVector v,
                                Data& data) override;

  const ConstraintBase& compute(
      double t, ConstRefVector q, ConstRefVector v, Data& data,
      const std::vector<std::shared_ptr<ContactLevel> >* contacts) override;

  const ConstraintBase& getConstraint() const override;

  void setReference(const Vector3& ref);

  const Vector3& getReference() const;

  void setContactNormal(const Vector3& n);

  const Vector3& getContactNormal() const;

 protected:
  const std::vector<std::shared_ptr<ContactLevel> >* m_contacts;
  std::string m_contact_name;  // an empty string
  Vector3 m_normal;  // normal direction to the ground expressed in world frame
  Vector3 m_ref;     // reference CoP in world frame
  ConstraintEquality m_constraint;
};

}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_com_equality_hpp__

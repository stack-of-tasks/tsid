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

#ifndef __invdyn_task_contact_force_hpp__
#define __invdyn_task_contact_force_hpp__

#include <tsid/deprecated.hh>
#include <tsid/tasks/task-base.hpp>
#include <tsid/formulations/contact-level.hpp>
#include <memory>

namespace tsid {
namespace tasks {
class TaskContactForce : public TaskBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TaskContactForce(const std::string& name, RobotWrapper& robot);

  /**
   * Contact force tasks have an additional compute method that takes as extra
   * input argument the list of active contacts. This can be needed for force
   * tasks that involve all contacts, such as the CoP task.
   */
  TSID_DISABLE_WARNING_PUSH
  // clang-format off
  TSID_DISABLE_WARNING(-Woverloaded-virtual)
  // clang-format on
  virtual const ConstraintBase& compute(
      const double t, ConstRefVector q, ConstRefVector v, Data& data,
      const std::vector<std::shared_ptr<ContactLevel> >* contacts) = 0;
  TSID_DISABLE_WARNING_POP

  /**
   * Return the name of the contact associated to this task if this task is
   * associated to a specific contact. If this task is associated to multiple
   * contact forces (all of them), returns an empty string.
   */
  virtual const std::string& getAssociatedContactName() = 0;
};
}  // namespace tasks
}  // namespace tsid

#endif  // ifndef __invdyn_task_contact_force_hpp__

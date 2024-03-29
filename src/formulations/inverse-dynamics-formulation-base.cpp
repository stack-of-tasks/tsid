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

#include "tsid/formulations/inverse-dynamics-formulation-base.hpp"

namespace tsid {

TaskLevel::TaskLevel(tasks::TaskBase& task, unsigned int priority)
    : task(task), priority(priority) {}

TaskLevelForce::TaskLevelForce(tasks::TaskContactForce& task,
                               unsigned int priority)
    : task(task), priority(priority) {}

InverseDynamicsFormulationBase::InverseDynamicsFormulationBase(
    const std::string& name, RobotWrapper& robot, bool verbose)
    : m_name(name), m_robot(robot), m_verbose(verbose) {}

MeasuredForceLevel::MeasuredForceLevel(
    contacts::MeasuredForceBase& measuredForce)
    : measuredForce(measuredForce) {}

bool InverseDynamicsFormulationBase::addRigidContact(ContactBase& contact) {
  return addRigidContact(contact, 1e-5);
}
}  // namespace tsid

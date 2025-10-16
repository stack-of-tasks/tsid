//
// Copyright (c) 2017 CNRS
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

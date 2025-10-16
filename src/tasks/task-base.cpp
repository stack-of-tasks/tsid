//
// Copyright (c) 2017 CNRS
//

#include "tsid/tasks/task-base.hpp"

namespace tsid {
namespace tasks {
TaskBase::TaskBase(const std::string& name, RobotWrapper& robot)
    : m_name(name), m_robot(robot) {}

const std::string& TaskBase::name() const { return m_name; }

void TaskBase::name(const std::string& name) { m_name = name; }

}  // namespace tasks
}  // namespace tsid

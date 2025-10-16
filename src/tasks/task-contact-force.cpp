//
// Copyright (c) 2017 CNRS
//

#include "tsid/tasks/task-contact-force.hpp"

namespace tsid {
namespace tasks {
using namespace tsid;

TaskContactForce::TaskContactForce(const std::string& name, RobotWrapper& robot)
    : TaskBase(name, robot) {}

}  // namespace tasks
}  // namespace tsid

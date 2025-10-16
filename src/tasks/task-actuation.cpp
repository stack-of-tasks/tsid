//
// Copyright (c) 2017 CNRS
//

#include <tsid/tasks/task-actuation.hpp>

namespace tsid {
namespace tasks {
using namespace tsid;

TaskActuation::TaskActuation(const std::string& name, RobotWrapper& robot)
    : TaskBase(name, robot) {}

}  // namespace tasks
}  // namespace tsid

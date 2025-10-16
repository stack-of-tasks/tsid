//
// Copyright (c) 2022 CNRS INRIA LORIA
//

#include "tsid/contacts/measured-force-base.hpp"

namespace tsid {
namespace contacts {
MeasuredForceBase::MeasuredForceBase(const std::string& name,
                                     RobotWrapper& robot)
    : m_name(name), m_robot(robot) {}

const std::string& MeasuredForceBase::name() const { return m_name; }

void MeasuredForceBase::name(const std::string& name) { m_name = name; }

}  // namespace contacts
}  // namespace tsid

//
// Copyright (c) 2017 CNRS
//

#include "tsid/contacts/contact-base.hpp"

namespace tsid {
namespace contacts {
ContactBase::ContactBase(const std::string& name, RobotWrapper& robot)
    : m_name(name), m_robot(robot) {}

const std::string& ContactBase::name() const { return m_name; }

void ContactBase::name(const std::string& name) { m_name = name; }

}  // namespace contacts
}  // namespace tsid

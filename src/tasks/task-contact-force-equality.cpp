//
// Copyright (c) 2017 CNRS
//

#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include "tsid/tasks/task-contact-force-equality.hpp"

namespace tsid {
namespace tasks {

using namespace tsid::math;
using namespace std;

TaskContactForceEquality::TaskContactForceEquality(
    const std::string& name, RobotWrapper& robot, const double dt,
    contacts::ContactBase& contact)
    : TaskContactForce(name, robot),
      m_contact(&contact),
      m_constraint(name, 6, 12),
      m_ref(6, 6),
      m_fext(6, 6) {
  m_forceIntegralError = Vector::Zero(6);
  m_dt = dt;
  m_leak_rate = 0.05;
  m_contact_name = m_contact->name();
}

int TaskContactForceEquality::dim() const { return 6; }

const Vector& TaskContactForceEquality::Kp() const { return m_Kp; }
const Vector& TaskContactForceEquality::Kd() const { return m_Kd; }
const Vector& TaskContactForceEquality::Ki() const { return m_Ki; }
const double& TaskContactForceEquality::getLeakRate() const {
  return m_leak_rate;
}

void TaskContactForceEquality::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == 6,
                                 "The size of the Kp vector needs to equal 6");
  m_Kp = Kp;
}

void TaskContactForceEquality::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == 6,
                                 "The size of the Kd vector needs to equal 6");
  m_Kd = Kd;
}

void TaskContactForceEquality::Ki(ConstRefVector Ki) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Ki.size() == 6,
                                 "The size of the Ki vector needs to equal 6");
  m_Ki = Ki;
}

void TaskContactForceEquality::setLeakRate(double leak) { m_leak_rate = leak; }

const std::string& TaskContactForceEquality::getAssociatedContactName() {
  return m_contact_name;
}

const contacts::ContactBase& TaskContactForceEquality::getAssociatedContact() {
  return *m_contact;
}

void TaskContactForceEquality::setAssociatedContact(
    contacts::ContactBase& contact) {
  m_contact = &contact;
  m_contact_name = m_contact->name();
}

void TaskContactForceEquality::setReference(TrajectorySample& ref) {
  m_ref = ref;
}

const TaskContactForceEquality::TrajectorySample&
TaskContactForceEquality::getReference() const {
  return m_ref;
}

void TaskContactForceEquality::setExternalForce(TrajectorySample& f_ext) {
  m_fext = f_ext;
}

const TaskContactForceEquality::TrajectorySample&
TaskContactForceEquality::getExternalForce() const {
  return m_fext;
}

const ConstraintBase& TaskContactForceEquality::compute(
    const double t, ConstRefVector q, ConstRefVector v, Data& data,
    const std::vector<std::shared_ptr<ContactLevel> >* contacts) {
  bool contactFound = false;
  if (m_contact_name != "") {
    // look if the associated contact is in the list of contact
    for (auto cl : *contacts) {
      if (m_contact_name == cl->contact.name()) {
        contactFound = true;
        break;
      }
    }
  } else {
    std::cout << "[TaskContactForceEquality] ERROR: Contact name empty"
              << std::endl;
    return m_constraint;
  }
  if (!contactFound) {
    std::cout << "[TaskContactForceEquality] ERROR: Contact name not in the "
                 "list of contact in the formulation pb"
              << std::endl;
    return m_constraint;
  }
  return compute(t, q, v, data);
}

const ConstraintBase& TaskContactForceEquality::compute(const double,
                                                        ConstRefVector,
                                                        ConstRefVector,
                                                        Data& /*data*/) {
  auto& M = m_constraint.matrix();
  M = m_contact->getForceGeneratorMatrix();  // 6x12 for a 6d contact

  Vector forceError = m_ref.getValue() - m_fext.getValue();
  Vector f_ref =
      m_ref.getValue() + m_Kp.cwiseProduct(forceError) +
      m_Kd.cwiseProduct(m_ref.getDerivative() - m_fext.getDerivative()) +
      m_Ki.cwiseProduct(m_forceIntegralError);
  m_constraint.vector() = f_ref;

  m_forceIntegralError +=
      (forceError - m_leak_rate * m_forceIntegralError) * m_dt;

  return m_constraint;
}

const ConstraintBase& TaskContactForceEquality::getConstraint() const {
  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid

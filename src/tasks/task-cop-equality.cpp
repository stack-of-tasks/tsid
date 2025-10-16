//
// Copyright (c) 2021 University of Trento
//

#include "tsid/tasks/task-cop-equality.hpp"

using namespace tsid::math;
using namespace std;

namespace tsid {
namespace tasks {

TaskCopEquality::TaskCopEquality(const std::string& name, RobotWrapper& robot)
    : TaskContactForce(name, robot),
      m_contact_name(""),
      m_constraint(name, 3, 3) {
  m_normal << 0, 0, 1;
  m_ref.setZero();
}

void TaskCopEquality::setContactList(
    const std::vector<std::shared_ptr<ContactLevel> >* contacts) {
  m_contacts = contacts;
}

int TaskCopEquality::dim() const { return 3; }

const std::string& TaskCopEquality::getAssociatedContactName() {
  return m_contact_name;
}

const ConstraintBase& TaskCopEquality::compute(
    const double t, ConstRefVector q, ConstRefVector v, Data& data,
    const std::vector<std::shared_ptr<ContactLevel> >* contacts) {
  setContactList(contacts);
  return compute(t, q, v, data);
}
const ConstraintBase& TaskCopEquality::compute(const double, ConstRefVector,
                                               ConstRefVector, Data& data) {
  // count size of force vector
  int n = 0;
  for (auto& cl : *m_contacts) {
    n += cl->contact.n_force();
  }

  // fill constraint matrix
  SE3 oMi;
  Vector3 p_local, p_world;
  auto& M = m_constraint.matrix();
  M.resize(3, n);
  for (auto& cl : *m_contacts) {
    unsigned int i = cl->index;
    // get contact points in local frame and transform them to world frame
    const Matrix3x& P = cl->contact.getContactPoints();
    m_robot.framePosition(
        data,
        static_cast<const TaskSE3Equality&>(cl->contact.getMotionTask())
            .frame_id(),
        oMi);
    // cout<<"Contact "<<cl->contact.name()<<endl;
    // cout<<"oMi\n"<<oMi<<endl;
    for (int j = 0; j < P.cols(); ++j) {
      p_local = P.col(j);
      p_world = oMi.act(p_local);
      // cout<<j<<" p_local "<<p_local.transpose()<<endl;
      // cout<<j<<" p_world "<<p_world.transpose()<<endl;
      M.middleCols(i + 3 * j, 3) = (p_world - m_ref) * (m_normal.transpose());
    }
  }
  return m_constraint;
}

const ConstraintBase& TaskCopEquality::getConstraint() const {
  return m_constraint;
}

void TaskCopEquality::setReference(const Vector3& ref) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(ref.size() == 3,
                                 "The size of the reference needs to equal 3");
  m_ref = ref;
}

const Vector3& TaskCopEquality::getReference() const { return m_ref; }

void TaskCopEquality::setContactNormal(const Vector3& n) { m_normal = n; }

const Vector3& TaskCopEquality::getContactNormal() const { return m_normal; }

}  // namespace tasks
}  // namespace tsid

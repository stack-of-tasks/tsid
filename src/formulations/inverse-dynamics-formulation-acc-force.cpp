//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
//

#include "tsid/formulations/inverse-dynamics-formulation-acc-force.hpp"

#include "tsid/math/constraint-bound.hpp"
#include "tsid/math/constraint-inequality.hpp"

using namespace tsid;
using namespace math;
using namespace tasks;
using namespace contacts;
using namespace solvers;
using namespace std;

typedef pinocchio::Data Data;

InverseDynamicsFormulationAccForce::InverseDynamicsFormulationAccForce(
    const std::string& name, RobotWrapper& robot, bool verbose)
    : InverseDynamicsFormulationBase(name, robot, verbose),
      m_data(robot.model()),
      m_baseDynamics(new math::ConstraintEquality(
          "base-dynamics", robot.nv() - robot.na(), robot.nv())),
      m_solutionDecoded(false) {
  m_t = 0.0;
  m_v = robot.nv();
  m_u = robot.nv() - robot.na();
  m_k = 0;
  m_eq = m_u;
  m_in = 0;
  m_hqpData.resize(2);
  m_Jc.setZero(m_k, m_v);
  h_fext.setZero(m_v);
  m_hqpData[0].push_back(
      solvers::make_pair<double, std::shared_ptr<ConstraintBase> >(
          1.0, m_baseDynamics));
}

Data& InverseDynamicsFormulationAccForce::data() { return m_data; }

unsigned int InverseDynamicsFormulationAccForce::nVar() const {
  return m_v + m_k;
}

unsigned int InverseDynamicsFormulationAccForce::nEq() const { return m_eq; }

unsigned int InverseDynamicsFormulationAccForce::nIn() const { return m_in; }

void InverseDynamicsFormulationAccForce::resizeHqpData() {
  m_Jc.setZero(m_k, m_v);
  m_baseDynamics->resize(m_u, m_v + m_k);
  for (HQPData::iterator it = m_hqpData.begin(); it != m_hqpData.end(); it++) {
    for (ConstraintLevel::iterator itt = it->begin(); itt != it->end(); itt++) {
      itt->second->resize(itt->second->rows(), m_v + m_k);
    }
  }
}

template <class TaskLevelPointer>
void InverseDynamicsFormulationAccForce::addTask(TaskLevelPointer tl,
                                                 double weight,
                                                 unsigned int priorityLevel) {
  if (priorityLevel > m_hqpData.size()) m_hqpData.resize(priorityLevel);
  const ConstraintBase& c = tl->task.getConstraint();
  if (c.isEquality()) {
    tl->constraint =
        std::make_shared<ConstraintEquality>(c.name(), c.rows(), m_v + m_k);
    if (priorityLevel == 0) m_eq += c.rows();
  } else  // if(c.isInequality())
  {
    tl->constraint =
        std::make_shared<ConstraintInequality>(c.name(), c.rows(), m_v + m_k);
    if (priorityLevel == 0) m_in += c.rows();
  }
  // don't use bounds for now because EiQuadProg doesn't exploit them anyway
  //  else
  //    tl->constraint = new ConstraintBound(c.name(), m_v+m_k);
  m_hqpData[priorityLevel].push_back(
      make_pair<double, std::shared_ptr<ConstraintBase> >(weight,
                                                          tl->constraint));
}

bool InverseDynamicsFormulationAccForce::addMotionTask(
    TaskMotion& task, double weight, unsigned int priorityLevel,
    double transition_duration) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      weight >= 0.0, "The weight needs to be positive or equal to 0");
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      transition_duration >= 0.0,
      "The transition duration needs to be greater than or equal to 0");

  auto tl = std::make_shared<TaskLevel>(task, priorityLevel);
  m_taskMotions.push_back(tl);
  addTask(tl, weight, priorityLevel);

  return true;
}

bool InverseDynamicsFormulationAccForce::addForceTask(
    TaskContactForce& task, double weight, unsigned int priorityLevel,
    double transition_duration) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      weight >= 0.0, "The weight needs to be positive or equal to 0");
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      transition_duration >= 0.0,
      "The transition duration needs to be greater than or equal to 0");

  auto tl = std::make_shared<TaskLevelForce>(task, priorityLevel);
  m_taskContactForces.push_back(tl);
  addTask(tl, weight, priorityLevel);
  return true;
}

bool InverseDynamicsFormulationAccForce::addActuationTask(
    TaskActuation& task, double weight, unsigned int priorityLevel,
    double transition_duration) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      weight >= 0.0, "The weight needs to be positive or equal to 0");
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      transition_duration >= 0.0,
      "The transition duration needs to be greater than or equal to 0");

  auto tl = std::make_shared<TaskLevel>(task, priorityLevel);
  m_taskActuations.push_back(tl);

  if (priorityLevel > m_hqpData.size()) m_hqpData.resize(priorityLevel);

  const ConstraintBase& c = tl->task.getConstraint();
  if (c.isEquality()) {
    tl->constraint =
        std::make_shared<ConstraintEquality>(c.name(), c.rows(), m_v + m_k);
    if (priorityLevel == 0) m_eq += c.rows();
  } else  // an actuator bound becomes an inequality because actuator forces are
          // not in the problem variables
  {
    tl->constraint =
        std::make_shared<ConstraintInequality>(c.name(), c.rows(), m_v + m_k);
    if (priorityLevel == 0) m_in += c.rows();
  }

  m_hqpData[priorityLevel].push_back(
      make_pair<double, std::shared_ptr<ConstraintBase> >(weight,
                                                          tl->constraint));

  return true;
}

bool InverseDynamicsFormulationAccForce::updateTaskWeight(
    const std::string& task_name, double weight) {
  ConstraintLevel::iterator it;
  // do not look into first priority level because weights do not matter there
  for (unsigned int i = 1; i < m_hqpData.size(); i++) {
    for (it = m_hqpData[i].begin(); it != m_hqpData[i].end(); it++) {
      if (it->second->name() == task_name) {
        it->first = weight;
        return true;
      }
    }
  }
  return false;
}

bool InverseDynamicsFormulationAccForce::addRigidContact(
    ContactBase& contact, double force_regularization_weight,
    double motion_weight, unsigned int motionPriorityLevel) {
  auto cl = std::make_shared<ContactLevel>(contact);
  cl->index = m_k;
  m_k += contact.n_force();
  m_contacts.push_back(cl);
  resizeHqpData();

  const ConstraintBase& motionConstr = contact.getMotionConstraint();
  cl->motionConstraint = std::make_shared<ConstraintEquality>(
      contact.name() + "_motion_task", motionConstr.rows(), m_v + m_k);
  m_hqpData[motionPriorityLevel].push_back(
      solvers::make_pair<double, std::shared_ptr<ConstraintBase> >(
          motion_weight, cl->motionConstraint));

  const ConstraintInequality& forceConstr = contact.getForceConstraint();
  cl->forceConstraint = std::make_shared<ConstraintInequality>(
      contact.name() + "_force_constraint", forceConstr.rows(), m_v + m_k);
  m_hqpData[0].push_back(
      solvers::make_pair<double, std::shared_ptr<ConstraintBase> >(
          1.0, cl->forceConstraint));

  const ConstraintEquality& forceRegConstr =
      contact.getForceRegularizationTask();
  cl->forceRegTask = std::make_shared<ConstraintEquality>(
      contact.name() + "_force_reg_task", forceRegConstr.rows(), m_v + m_k);
  m_hqpData[1].push_back(
      solvers::make_pair<double, std::shared_ptr<ConstraintBase> >(
          force_regularization_weight, cl->forceRegTask));

  if (motionPriorityLevel == 0) m_eq += motionConstr.rows();
  m_in += forceConstr.rows();

  return true;
}

bool InverseDynamicsFormulationAccForce::addRigidContact(ContactBase& contact) {
  std::cout << "[InverseDynamicsFormulationAccForce] Method "
               "addRigidContact(ContactBase) is deprecated. You should use "
               "addRigidContact(ContactBase, double) instead.\n";
  return addRigidContact(contact, 1e-5);
}

bool InverseDynamicsFormulationAccForce::updateRigidContactWeights(
    const std::string& contact_name, double force_regularization_weight,
    double motion_weight) {
  // update weight of force regularization task
  ConstraintLevel::iterator itt;
  bool force_reg_task_found = false;
  bool motion_task_found = false;
  for (unsigned int i = 1; i < m_hqpData.size(); i++) {
    for (itt = m_hqpData[i].begin(); itt != m_hqpData[i].end(); itt++) {
      if (itt->second->name() == contact_name + "_force_reg_task") {
        if (force_regularization_weight >= 0.0)
          itt->first = force_regularization_weight;
        if (motion_task_found || motion_weight < 0.0)
          return true;  // If motion_weight is negative, the motion_task will
                        // not be modified. The method can return here
        force_reg_task_found = true;
      } else if (itt->second->name() == contact_name + "_motion_task") {
        if (motion_weight >= 0.0) itt->first = motion_weight;
        if (force_reg_task_found) return true;
        motion_task_found = true;
      }
    }
  }
  return false;
}

bool InverseDynamicsFormulationAccForce::addMeasuredForce(
    MeasuredForceBase& measuredForce) {
  auto tl = std::make_shared<MeasuredForceLevel>(measuredForce);
  m_measuredForces.push_back(tl);

  return true;
}

const HQPData& InverseDynamicsFormulationAccForce::computeProblemData(
    double time, ConstRefVector q, ConstRefVector v) {
  m_t = time;

  for (auto it_ct = m_contactTransitions.begin();
       it_ct != m_contactTransitions.end(); it_ct++) {
    auto c = *it_ct;
    assert(c->time_start <= m_t);
    if (m_t <= c->time_end) {
      const double alpha =
          (m_t - c->time_start) / (c->time_end - c->time_start);
      const double fMax = c->fMax_start + alpha * (c->fMax_end - c->fMax_start);
      c->contactLevel->contact.setMaxNormalForce(fMax);
    } else {
      // std::cout<<"[InverseDynamicsFormulationAccForce] Remove contact "<<
      //            c->contactLevel->contact.name()<<" at time
      //            "<<time<<std::endl;
      removeRigidContact(c->contactLevel->contact.name());
      // FIXME: this won't work if multiple contact transitions occur at the
      // same time because after erasing an element the iterator is invalid
      m_contactTransitions.erase(it_ct);
      break;
    }
  }

  m_robot.computeAllTerms(m_data, q, v);

  for (auto cl : m_contacts) {
    unsigned int m = cl->contact.n_force();

    const ConstraintBase& mc =
        cl->contact.computeMotionTask(time, q, v, m_data);
    cl->motionConstraint->matrix().leftCols(m_v) = mc.matrix();
    cl->motionConstraint->vector() = mc.vector();

    const Matrix& T =
        cl->contact.getForceGeneratorMatrix();  // e.g., 6x12 for a 6d contact
    m_Jc.middleRows(cl->index, m).noalias() = T.transpose() * mc.matrix();

    const ConstraintInequality& fc =
        cl->contact.computeForceTask(time, q, v, m_data);
    cl->forceConstraint->matrix().middleCols(m_v + cl->index, m) = fc.matrix();
    cl->forceConstraint->lowerBound() = fc.lowerBound();
    cl->forceConstraint->upperBound() = fc.upperBound();

    const ConstraintEquality& fr =
        cl->contact.computeForceRegularizationTask(time, q, v, m_data);
    cl->forceRegTask->matrix().middleCols(m_v + cl->index, m) = fr.matrix();
    cl->forceRegTask->vector() = fr.vector();
  }

  // Add all measured external forces to dynamic model
  h_fext.setZero(m_v);
  for (auto it : m_measuredForces) {
    h_fext += it->measuredForce.computeJointTorques(m_data);
  }

  const Matrix& M_a = m_robot.mass(m_data).bottomRows(m_v - m_u);
  const Vector& h_a =
      m_robot.nonLinearEffects(m_data).tail(m_v - m_u) - h_fext.tail(m_v - m_u);
  const Matrix& J_a = m_Jc.rightCols(m_v - m_u);
  const Matrix& M_u = m_robot.mass(m_data).topRows(m_u);
  const Vector& h_u =
      m_robot.nonLinearEffects(m_data).head(m_u) - h_fext.head(m_u);
  const Matrix& J_u = m_Jc.leftCols(m_u);

  m_baseDynamics->matrix().leftCols(m_v) = M_u;
  m_baseDynamics->matrix().rightCols(m_k) = -J_u.transpose();
  m_baseDynamics->vector() = -h_u;

  //  std::vector<TaskLevel*>::iterator it;
  //  for(it=m_taskMotions.begin(); it!=m_taskMotions.end(); it++)
  for (auto& it : m_taskMotions) {
    const ConstraintBase& c = it->task.compute(time, q, v, m_data);
    if (c.isEquality()) {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->vector() = c.vector();
    } else if (c.isInequality()) {
      it->constraint->matrix().leftCols(m_v) = c.matrix();
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    } else {
      it->constraint->matrix().leftCols(m_v) = Matrix::Identity(m_v, m_v);
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
  }

  for (auto& it : m_taskContactForces) {
    // cout<<"Task "<<it->task.name()<<endl;
    // by default the task is associated to all contact forces
    int i0 = m_v;
    int c_size = m_k;

    // if the task is associated to a specific contact
    // cout<<"Associated contact name:
    // "<<it->task.getAssociatedContactName()<<endl;
    if (it->task.getAssociatedContactName() != "") {
      // look for the associated contact
      for (auto cl : m_contacts) {
        if (it->task.getAssociatedContactName() == cl->contact.name()) {
          i0 += cl->index;
          c_size = cl->contact.n_force();
          break;
        }
      }
    }

    const ConstraintBase& c = it->task.compute(time, q, v, m_data, &m_contacts);
    // cout<<"matrix"<<endl<<c.matrix()<<endl;
    // cout<<"vector"<<endl<<c.vector().transpose()<<endl;
    // cout<<"i0 "<<i0<<" c_size "<<c_size<<endl;
    // cout<<"constraint matrix size: "<<it->constraint->matrix().rows()<<" x
    // "<<it->constraint->matrix().cols()<<endl;

    if (c.isEquality()) {
      it->constraint->matrix().middleCols(i0, c_size) = c.matrix();
      it->constraint->vector() = c.vector();
    } else if (c.isInequality()) {
      it->constraint->matrix().middleCols(i0, c_size) = c.matrix();
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    } else {
      it->constraint->matrix().middleCols(i0, c_size) =
          Matrix::Identity(c_size, c_size);
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->upperBound() = c.upperBound();
    }
  }

  for (auto& it : m_taskActuations) {
    const ConstraintBase& c = it->task.compute(time, q, v, m_data);
    if (c.isEquality()) {
      it->constraint->matrix().leftCols(m_v).noalias() = c.matrix() * M_a;
      it->constraint->matrix().rightCols(m_k).noalias() =
          -c.matrix() * J_a.transpose();
      it->constraint->vector() = c.vector();
      it->constraint->vector().noalias() -= c.matrix() * h_a;
    } else if (c.isInequality()) {
      it->constraint->matrix().leftCols(m_v).noalias() = c.matrix() * M_a;
      it->constraint->matrix().rightCols(m_k).noalias() =
          -c.matrix() * J_a.transpose();
      it->constraint->lowerBound() = c.lowerBound();
      it->constraint->lowerBound().noalias() -= c.matrix() * h_a;
      it->constraint->upperBound() = c.upperBound();
      it->constraint->upperBound().noalias() -= c.matrix() * h_a;
    } else {
      // NB: An actuator bound becomes an inequality
      it->constraint->matrix().leftCols(m_v) = M_a;
      it->constraint->matrix().rightCols(m_k) = -J_a.transpose();
      it->constraint->lowerBound() = c.lowerBound() - h_a;
      it->constraint->upperBound() = c.upperBound() - h_a;
    }
  }

  m_solutionDecoded = false;

  return m_hqpData;
}

bool InverseDynamicsFormulationAccForce::decodeSolution(const HQPOutput& sol) {
  if (m_solutionDecoded) return true;

  const Matrix& M_a = m_robot.mass(m_data).bottomRows(m_v - m_u);
  const Vector& h_a =
      m_robot.nonLinearEffects(m_data).tail(m_v - m_u) - h_fext.tail(m_v - m_u);
  const Matrix& J_a = m_Jc.rightCols(m_v - m_u);
  m_dv = sol.x.head(m_v);
  m_f = sol.x.tail(m_k);
  m_tau = h_a;
  m_tau.noalias() += M_a * m_dv;
  m_tau.noalias() -= J_a.transpose() * m_f;
  m_solutionDecoded = true;
  return true;
}

const Vector& InverseDynamicsFormulationAccForce::getActuatorForces(
    const HQPOutput& sol) {
  decodeSolution(sol);
  return m_tau;
}

const Vector& InverseDynamicsFormulationAccForce::getAccelerations(
    const HQPOutput& sol) {
  decodeSolution(sol);
  return m_dv;
}

const Vector& InverseDynamicsFormulationAccForce::getContactForces(
    const HQPOutput& sol) {
  decodeSolution(sol);
  return m_f;
}

Vector InverseDynamicsFormulationAccForce::getContactForces(
    const std::string& name, const HQPOutput& sol) {
  decodeSolution(sol);
  // for(std::vector<ContactLevel*>::iterator it=m_contacts.begin();
  // it!=m_contacts.end(); it++)
  for (auto& it : m_contacts) {
    if (it->contact.name() == name) {
      const int k = it->contact.n_force();
      return m_f.segment(it->index, k);
    }
  }
  return Vector::Zero(0);
}

bool InverseDynamicsFormulationAccForce::getContactForces(
    const std::string& name, const HQPOutput& sol, RefVector f) {
  decodeSolution(sol);
  for (auto& it : m_contacts) {
    if (it->contact.name() == name) {
      const int k = it->contact.n_force();
      assert(f.size() == k);
      f = m_f.segment(it->index, k);
      return true;
    }
  }
  return false;
}

bool InverseDynamicsFormulationAccForce::removeTask(const std::string& taskName,
                                                    double) {
#ifndef NDEBUG
  bool taskFound = removeFromHqpData(taskName);
  assert(taskFound);
#else
  removeFromHqpData(taskName);
#endif

  for (auto it = m_taskMotions.begin(); it != m_taskMotions.end(); it++) {
    if ((*it)->task.name() == taskName) {
      if ((*it)->priority == 0) {
        if ((*it)->constraint->isEquality())
          m_eq -= (*it)->constraint->rows();
        else if ((*it)->constraint->isInequality())
          m_in -= (*it)->constraint->rows();
      }
      m_taskMotions.erase(it);
      return true;
    }
  }
  for (auto it = m_taskContactForces.begin(); it != m_taskContactForces.end();
       it++) {
    if ((*it)->task.name() == taskName) {
      m_taskContactForces.erase(it);
      return true;
    }
  }
  for (auto it = m_taskActuations.begin(); it != m_taskActuations.end(); it++) {
    if ((*it)->task.name() == taskName) {
      if ((*it)->priority == 0) {
        if ((*it)->constraint->isEquality())
          m_eq -= (*it)->constraint->rows();
        else
          m_in -= (*it)->constraint->rows();
      }
      m_taskActuations.erase(it);
      return true;
    }
  }
  return false;
}

bool InverseDynamicsFormulationAccForce::removeRigidContact(
    const std::string& contactName, double transition_duration) {
  if (transition_duration > 0.0) {
    for (auto& it : m_contacts) {
      if (it->contact.name() == contactName) {
        auto transitionInfo = std::make_shared<ContactTransitionInfo>();
        transitionInfo->contactLevel = it;
        transitionInfo->time_start = m_t;
        transitionInfo->time_end = m_t + transition_duration;
        const int k = it->contact.n_force();
        if (m_f.size() >= it->index + k) {
          const Vector f = m_f.segment(it->index, k);
          transitionInfo->fMax_start = it->contact.getNormalForce(f);
        } else {
          transitionInfo->fMax_start = it->contact.getMaxNormalForce();
        }
        transitionInfo->fMax_end = it->contact.getMinNormalForce() + 1e-3;
        m_contactTransitions.push_back(transitionInfo);
        return true;
      }
    }
    return false;
  }

  // Find motion task priority
  bool is_motion_contraint =
      (getTaskPriority(contactName + "_motion_task") == 0);
  bool first_constraint_found = removeFromHqpData(contactName + "_motion_task");
  assert(first_constraint_found);

  bool second_constraint_found =
      removeFromHqpData(contactName + "_force_constraint");
  assert(second_constraint_found);

  bool third_constraint_found =
      removeFromHqpData(contactName + "_force_reg_task");
  assert(third_constraint_found);

  bool contact_found = false;
  for (auto it = m_contacts.begin(); it != m_contacts.end(); it++) {
    if ((*it)->contact.name() == contactName) {
      m_k -= (*it)->contact.n_force();
      if (is_motion_contraint) m_eq -= (*it)->motionConstraint->rows();
      m_in -= (*it)->forceConstraint->rows();
      m_contacts.erase(it);
      resizeHqpData();
      contact_found = true;
      break;
    }
  }

  int k = 0;
  for (auto& it : m_contacts) {
    it->index = k;
    k += it->contact.n_force();
  }
  return contact_found && first_constraint_found && second_constraint_found &&
         third_constraint_found;
}

bool InverseDynamicsFormulationAccForce::removeMeasuredForce(
    const std::string& measuredForceName) {
  for (auto it = m_measuredForces.begin(); it != m_measuredForces.end(); it++) {
    if ((*it)->measuredForce.name() == measuredForceName) {
      m_measuredForces.erase(it);
      return true;
    }
  }
  return false;
}

bool InverseDynamicsFormulationAccForce::removeFromHqpData(
    const std::string& name) {
  bool found = false;
  for (HQPData::iterator it = m_hqpData.begin();
       !found && it != m_hqpData.end(); it++) {
    for (ConstraintLevel::iterator itt = it->begin();
         !found && itt != it->end(); itt++) {
      if (itt->second->name() == name) {
        it->erase(itt);
        return true;
      }
    }
  }
  return false;
}

unsigned int InverseDynamicsFormulationAccForce::getTaskPriority(
    const std::string& name) {
  for (std::size_t i = 0; i < m_hqpData.size(); i++) {
    const bool found = std::any_of(
        m_hqpData[i].begin(), m_hqpData[i].end(),
        [&name](const auto& c) { return c.second->name() == name; });
    if (found) {
      return i;
    }
  }
  assert(false);  // Task name not found in formulation data
  return -1;
}

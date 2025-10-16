//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen, UNITN
//

#ifndef __invdyn_inverse_dynamics_formulation_acc_force_hpp__
#define __invdyn_inverse_dynamics_formulation_acc_force_hpp__

#include <vector>

#include "tsid/formulations/contact-level.hpp"
#include "tsid/formulations/inverse-dynamics-formulation-base.hpp"
#include "tsid/math/constraint-equality.hpp"

namespace tsid {

class ContactTransitionInfo {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time_start;
  double time_end;
  double fMax_start;  /// max normal force at time time_start
  double fMax_end;    /// max normal force at time time_end
  std::shared_ptr<ContactLevel> contactLevel;
};

class InverseDynamicsFormulationAccForce
    : public InverseDynamicsFormulationBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef pinocchio::Data Data;
  typedef math::Vector Vector;
  typedef math::Matrix Matrix;
  typedef math::ConstRefVector ConstRefVector;
  typedef tasks::TaskBase TaskBase;
  typedef tasks::TaskMotion TaskMotion;
  typedef tasks::TaskContactForce TaskContactForce;
  typedef tasks::TaskActuation TaskActuation;
  typedef contacts::MeasuredForceBase MeasuredForceBase;
  typedef solvers::HQPOutput HQPOutput;

  InverseDynamicsFormulationAccForce(const std::string& name,
                                     RobotWrapper& robot, bool verbose = false);

  Data& data() override;

  unsigned int nVar() const override;
  unsigned int nEq() const override;
  unsigned int nIn() const override;

  bool addMotionTask(TaskMotion& task, double weight,
                     unsigned int priorityLevel,
                     double transition_duration = 0.0) override;

  bool addForceTask(TaskContactForce& task, double weight,
                    unsigned int priorityLevel,
                    double transition_duration = 0.0) override;

  bool addActuationTask(TaskActuation& task, double weight,
                        unsigned int priorityLevel,
                        double transition_duration = 0.0) override;

  bool updateTaskWeight(const std::string& task_name, double weight) override;

  bool addRigidContact(ContactBase& contact, double force_regularization_weight,
                       double motion_weight = 1.0,
                       unsigned int motion_priority_level = 0) override;

  TSID_DEPRECATED bool addRigidContact(ContactBase& contact) override;

  bool updateRigidContactWeights(const std::string& contact_name,
                                 double force_regularization_weight,
                                 double motion_weight = -1.0) override;

  bool addMeasuredForce(MeasuredForceBase& measuredForce) override;

  bool removeTask(const std::string& taskName,
                  double transition_duration = 0.0) override;

  bool removeRigidContact(const std::string& contactName,
                          double transition_duration = 0.0) override;

  bool removeMeasuredForce(const std::string& measuredForceName) override;

  const HQPData& computeProblemData(double time, ConstRefVector q,
                                    ConstRefVector v) override;

  const Vector& getActuatorForces(const HQPOutput& sol) override;
  const Vector& getAccelerations(const HQPOutput& sol) override;
  const Vector& getContactForces(const HQPOutput& sol) override;
  Vector getContactForces(const std::string& name, const HQPOutput& sol);
  bool getContactForces(const std::string& name, const HQPOutput& sol,
                        RefVector f) override;
  unsigned int getTaskPriority(const std::string& name) override;

 public:
  template <class TaskLevelPointer>
  void addTask(TaskLevelPointer task, double weight,
               unsigned int priorityLevel);

  void resizeHqpData();

  bool removeFromHqpData(const std::string& name);

  bool decodeSolution(const HQPOutput& sol);

  Data m_data;
  HQPData m_hqpData;
  std::vector<std::shared_ptr<TaskLevel>> m_taskMotions;
  std::vector<std::shared_ptr<TaskLevelForce>> m_taskContactForces;
  std::vector<std::shared_ptr<TaskLevel>> m_taskActuations;
  std::vector<std::shared_ptr<ContactLevel>> m_contacts;
  std::vector<std::shared_ptr<MeasuredForceLevel>> m_measuredForces;
  double m_t;         /// time
  unsigned int m_k;   /// number of contact-force variables
  unsigned int m_v;   /// number of acceleration variables
  unsigned int m_u;   /// number of unactuated DoFs
  unsigned int m_eq;  /// number of equality constraints
  unsigned int m_in;  /// number of inequality constraints
  Matrix m_Jc;        /// contact force Jacobian
  std::shared_ptr<math::ConstraintEquality> m_baseDynamics;

  bool m_solutionDecoded;
  Vector m_dv;
  Vector m_f;
  Vector m_tau;

  Vector h_fext;  /// sum of external measured forces

  std::vector<std::shared_ptr<ContactTransitionInfo>> m_contactTransitions;
};
}  // namespace tsid
#endif  // ifndef __invdyn_inverse_dynamics_formulation_acc_force_hpp__

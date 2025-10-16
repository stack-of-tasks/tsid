//
// Copyright (c) 2023 MIPT
//

#ifndef __invdyn_contact_two_frame_positions_hpp__
#define __invdyn_contact_two_frame_positions_hpp__

#include "tsid/contacts/contact-base.hpp"
#include "tsid/tasks/task-se3-equality.hpp"
#include "tsid/tasks/task-two-frames-equality.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"

namespace tsid {
namespace contacts {
class ContactTwoFramePositions : public ContactBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::ConstRefMatrix ConstRefMatrix;
  typedef math::ConstRefVector ConstRefVector;
  typedef math::Matrix3x Matrix3x;
  typedef math::Vector6 Vector6;
  typedef math::Vector3 Vector3;
  typedef math::Vector Vector;
  typedef tasks::TaskTwoFramesEquality TaskTwoFramesEquality;
  typedef tasks::TaskSE3Equality TaskSE3Equality;
  typedef math::ConstraintInequality ConstraintInequality;
  typedef math::ConstraintEquality ConstraintEquality;
  typedef pinocchio::SE3 SE3;

  ContactTwoFramePositions(const std::string& name, RobotWrapper& robot,
                           const std::string& frameName1,
                           const std::string& frameName2,
                           const double minNormalForce,
                           const double maxNormalForce);

  /// Return the number of motion constraints
  unsigned int n_motion() const override;

  /// Return the number of force variables
  unsigned int n_force() const override;

  const ConstraintBase& computeMotionTask(double t, ConstRefVector q,
                                          ConstRefVector v,
                                          Data& data) override;

  const ConstraintInequality& computeForceTask(double t, ConstRefVector q,
                                               ConstRefVector v,
                                               const Data& data) override;

  const Matrix& getForceGeneratorMatrix() override;

  const ConstraintEquality& computeForceRegularizationTask(
      double t, ConstRefVector q, ConstRefVector v, const Data& data) override;

  const TaskTwoFramesEquality& getMotionTask() const override;
  const ConstraintBase& getMotionConstraint() const override;
  const ConstraintInequality& getForceConstraint() const override;
  const ConstraintEquality& getForceRegularizationTask() const override;
  double getMotionTaskWeight() const;
  const Matrix3x& getContactPoints() const override;

  double getNormalForce(ConstRefVector f) const override;
  double getMinNormalForce() const override;
  double getMaxNormalForce() const override;

  const Vector&
  Kp();  // cannot be const because it set a member variable inside
  const Vector&
  Kd();  // cannot be const because it set a member variable inside
  void Kp(ConstRefVector Kp);
  void Kd(ConstRefVector Kp);

  bool setContactNormal(ConstRefVector contactNormal);

  bool setFrictionCoefficient(const double frictionCoefficient);
  bool setMinNormalForce(const double minNormalForce) override;
  bool setMaxNormalForce(const double maxNormalForce) override;
  bool setMotionTaskWeight(const double w);
  void setForceReference(ConstRefVector& f_ref);
  void setRegularizationTaskWeightVector(ConstRefVector& w);

 protected:
  void updateForceInequalityConstraints();
  void updateForceRegularizationTask();
  void updateForceGeneratorMatrix();

  TaskTwoFramesEquality m_motionTask;
  ConstraintInequality m_forceInequality;
  ConstraintEquality m_forceRegTask;
  Vector3 m_fRef;
  Vector3 m_weightForceRegTask;
  Matrix3x m_contactPoints;
  Vector m_Kp3, m_Kd3;  // gain vectors to be returned by reference
  double m_fMin;
  double m_fMax;
  double m_regularizationTaskWeight;
  double m_motionTaskWeight;
  Matrix m_forceGenMat;
};
}  // namespace contacts
}  // namespace tsid

#endif  // ifndef __invdyn_contact_two_frame_positions_hpp__

//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#include <pininvdyn/contacts/contact-6d.hpp>

using namespace pininvdyn;
using namespace pininvdyn::contacts;
using namespace pininvdyn::math;
using namespace pininvdyn::trajectories;
using namespace pininvdyn::tasks;

Contact6d::Contact6d(const std::string & name,
                     RobotWrapper & robot,
                     const std::string & frameName,
                     ConstRefMatrix contactPoints,
                     ConstRefVector contactNormal,
                     const double frictionCoefficient,
                     const double minNormalForce,
                     const double regularizationTaskWeight):
  ContactBase(name, robot),
  m_motionTask(name, robot, frameName),
  m_forceInequality(name, 16, 12),
  m_forceRegTask(name, 12, 12),
  m_contactPoints(contactPoints),
  m_contactNormal(contactNormal),
  m_mu(frictionCoefficient),
  m_fMin(minNormalForce),
  m_regularizationTaskWeight(regularizationTaskWeight)
{
  m_forceGenMat.resize(6,12);
  updateForceGeneratorMatrix();
  updateForceInequalityConstraints();
  updateForceRegularizationTask();
}

void Contact6d::updateForceInequalityConstraints()
{
  Vector3 t1, t2;
  Matrix B = Matrix::Zero(5*4, 3*4);
  Vector lb = -1e10*Vector::Ones(5*4);
  Vector ub =  1e10*Vector::Ones(5*4);
  t1 = m_contactNormal.cross(Vector3::UnitX());
  if(t1.norm()<1e-5)
    t1 = m_contactNormal.cross(Vector3::UnitY());
  t2 = m_contactNormal.cross(t1);
  t1.normalize();
  t2.normalize();

  B.block<1,3>(0,0) = (-t1 - m_mu*m_contactNormal).transpose();
  B.block<1,3>(1,0) = (t1 - m_mu*m_contactNormal).transpose();
  B.block<1,3>(2,0) = (-t2 - m_mu*m_contactNormal).transpose();
  B.block<1,3>(3,0) = (t2 - m_mu*m_contactNormal).transpose();
  ub.head<4>().setZero();
  B.block<1,3>(4,0) = m_contactNormal.transpose();
  lb(4)    = m_fMin;

  for(int i=1; i<4; i++)
  {
    B.block<5,3>(5*i,3*i)   = B.topLeftCorner<5,3>();
    lb.segment<5>(5*i)      = lb.head<5>();
    ub.segment<5>(5*i)      = ub.head<5>();
  }

  m_forceInequality.setMatrix(B);
  m_forceInequality.setLowerBound(lb);
  m_forceInequality.setUpperBound(ub);
}

void Contact6d::updateForceRegularizationTask()
{
  // [1, 1, 1e-3, 2, 2, 2] weights of force regularization task
typedef Eigen::Matrix<double,6,6> Matrix6;
typedef Eigen::Matrix<double,12,1> Vector12;
  Vector6 m_weightForceRegTask;
  m_weightForceRegTask << 1, 1, 1e-3, 2, 2, 2;
  Matrix6 A = Matrix6::Zero();
  A.diagonal() = m_weightForceRegTask;
  m_forceRegTask.setMatrix(A*m_forceGenMat);
  m_forceRegTask.setVector(Vector6::Zero());
}

void Contact6d:: updateForceGeneratorMatrix()
{
  assert(m_contactPoints.rows()==3);
  assert(m_contactPoints.cols()==4);
  for(int i=0; i<4; i++)
  {
    m_forceGenMat.block<3,3>(0, i*3).setIdentity();
    m_forceGenMat.block<3,3>(3, i*3) = skew(m_contactPoints.col(i));
  }
}

unsigned int Contact6d::n_motion() const { return 6; }
unsigned int Contact6d::n_force() const { return 12; }

const Vector & Contact6d::Kp() const { return m_motionTask.Kp(); }
const Vector & Contact6d::Kd() const { return m_motionTask.Kd(); }
void Contact6d::Kp(ConstRefVector Kp){ m_motionTask.Kp(Kp); }
void Contact6d::Kd(ConstRefVector Kd){ m_motionTask.Kd(Kd); }

bool Contact6d::setContactPoints(ConstRefMatrix contactPoints)
{
  assert(contactPoints.rows()==3);
  assert(contactPoints.cols()==4);
  if(contactPoints.rows()!=3 || contactPoints.cols()!=4)
    return false;
  m_contactPoints = contactPoints;
  updateForceGeneratorMatrix();
  return true;
}

bool Contact6d::setContactNormal(ConstRefVector contactNormal)
{
  assert(contactNormal.size()==3);
  if(contactNormal.size()!=3)
    return false;
  m_contactNormal = contactNormal;
  updateForceInequalityConstraints();
  return true;
}

bool Contact6d::setFrictionCoefficient(const double frictionCoefficient)
{
  assert(frictionCoefficient>0.0);
  if(frictionCoefficient<=0.0)
    return false;
  m_mu = frictionCoefficient;
  updateForceInequalityConstraints();
  return true;
}

bool Contact6d::setMinNormalForce(const double minNormalForce)
{
  assert(minNormalForce>0.0);
  if(minNormalForce<=0.0)
    return false;
  m_fMin = minNormalForce;
  updateForceInequalityConstraints();
  return true;
}

bool Contact6d::setRegularizationTaskWeight(const double w)
{
  assert(w>=0.0);
  if(w<0.0)
    return false;
  m_regularizationTaskWeight = w;
  return true;
}

void Contact6d::setReference(const SE3 & ref)
{
  TrajectorySample s(12, 6);
  se3ToVector(ref, s.pos);
  m_motionTask.setReference(s);
}

const ConstraintBase & Contact6d::computeMotionTask(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
{
  return m_motionTask.compute(t, q, v, data);
}

const ConstraintInequality & Contact6d::computeForceTask(const double t,
                                                         ConstRefVector q,
                                                         ConstRefVector v,
                                                         Data & data)
{
  return m_forceInequality;
}

const Matrix & Contact6d::getForceGeneratorMatrix()
{
  return m_forceGenMat;
}

const ConstraintEquality & Contact6d::computeForceRegularizationTask(const double t,
                                                                     ConstRefVector q,
                                                                     ConstRefVector v,
                                                                     Data & data)
{
  return m_forceRegTask;
}

const TaskMotion & Contact6d::getMotionTask() const { return m_motionTask; }

const ConstraintBase & Contact6d::getMotionConstraint() const { return m_motionTask.getConstraint(); }

const ConstraintInequality & Contact6d::getForceConstraint() const { return m_forceInequality; }

const ConstraintEquality & Contact6d::getForceRegularizationTask() const { return m_forceRegTask; }

double Contact6d::getForceRegularizationWeight() const { return m_regularizationTaskWeight; }

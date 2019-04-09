//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#include "tsid/math/utils.hpp"
#include "tsid/contacts/contact-6d.hpp"

#include <pinocchio/spatial/skew.hpp>

using namespace tsid;
using namespace contacts;
using namespace math;
using namespace trajectories;
using namespace tasks;

Contact6d::Contact6d(const std::string & name,
                     RobotWrapper & robot,
                     const std::string & frameName,
                     ConstRefMatrix contactPoints,
                     ConstRefVector contactNormal,
                     const double frictionCoefficient,
                     const double minNormalForce,
                     const double maxNormalForce):
  ContactBase(name, robot),
  m_motionTask(name, robot, frameName),
  m_forceInequality(name, 17, 12),
  m_forceRegTask(name, 6, 12),
  m_contactPoints(contactPoints),
  m_contactNormal(contactNormal),
  m_mu(frictionCoefficient),
  m_fMin(minNormalForce),
  m_fMax(maxNormalForce)
{
  this->init();
}

Contact6d::Contact6d(const std::string & name,
                     RobotWrapper & robot,
                     const std::string & frameName,
                     ConstRefMatrix contactPoints,
                     ConstRefVector contactNormal,
                     const double frictionCoefficient,
                     const double minNormalForce,
                     const double maxNormalForce,
                     const double ):
  ContactBase(name, robot),
  m_motionTask(name, robot, frameName),
  m_forceInequality(name, 17, 12),
  m_forceRegTask(name, 6, 12),
  m_contactPoints(contactPoints),
  m_contactNormal(contactNormal),
  m_mu(frictionCoefficient),
  m_fMin(minNormalForce),
  m_fMax(maxNormalForce)
{
  std::cout<<"[Contact6d] The constructor with forceRegWeight is deprecated now. forceRegWeight should now be specified when calling addRigidContact()\n";
  this->init();
}

void Contact6d::init()
{
  m_weightForceRegTask << 1, 1, 1e-3, 2, 2, 2;
  m_forceGenMat.resize(6,12);
  m_fRef = Vector6::Zero();
  updateForceGeneratorMatrix();
  updateForceInequalityConstraints();
  updateForceRegularizationTask();
}

void Contact6d::updateForceInequalityConstraints()
{
  Vector3 t1, t2;
  const int n_in = 4*4 + 1;
  const int n_var = 3*4;
  Matrix B = Matrix::Zero(n_in, n_var);
  Vector lb = -1e10*Vector::Ones(n_in);
  Vector ub =  Vector::Zero(n_in);
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

  for(int i=1; i<4; i++)
  {
    B.block<4,3>(4*i,3*i)   = B.topLeftCorner<4,3>();
  }

  B.block<1,3>(n_in-1,0) = m_contactNormal.transpose();
  B.block<1,3>(n_in-1,3) = m_contactNormal.transpose();
  B.block<1,3>(n_in-1,6) = m_contactNormal.transpose();
  B.block<1,3>(n_in-1,9) = m_contactNormal.transpose();
  ub(n_in-1)    = m_fMax;
  lb(n_in-1)    = m_fMin;

  m_forceInequality.setMatrix(B);
  m_forceInequality.setLowerBound(lb);
  m_forceInequality.setUpperBound(ub);
}

double Contact6d::getNormalForce(ConstRefVector f) const
{
  assert(f.size()==n_force());
  double n=0.0;
  for(int i=0; i<4; i++)
    n += m_contactNormal.dot(f.segment<3>(i*3));
  return n;
}

void Contact6d::setRegularizationTaskWeightVector(ConstRefVector & w)
{
  m_weightForceRegTask = w;
  updateForceRegularizationTask();
}

void Contact6d::updateForceRegularizationTask()
{
typedef Eigen::Matrix<double,6,6> Matrix6;
  Matrix6 A = Matrix6::Zero();
  A.diagonal() = m_weightForceRegTask;
  m_forceRegTask.setMatrix(A*m_forceGenMat);
  m_forceRegTask.setVector(m_fRef);
}

void Contact6d:: updateForceGeneratorMatrix()
{
  assert(m_contactPoints.rows()==3);
  assert(m_contactPoints.cols()==4);
  for(int i=0; i<4; i++)
  {
    m_forceGenMat.block<3,3>(0, i*3).setIdentity();
    m_forceGenMat.block<3,3>(3, i*3) = pinocchio::skew(m_contactPoints.col(i));
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
  assert(minNormalForce>0.0 && minNormalForce<=m_fMax);
  if(minNormalForce<=0.0 || minNormalForce>m_fMax)
    return false;
  m_fMin = minNormalForce;
  Vector & lb = m_forceInequality.lowerBound();
  lb(lb.size()-1) = m_fMin;
  return true;
}

bool Contact6d::setMaxNormalForce(const double maxNormalForce)
{
  assert(maxNormalForce>=m_fMin);
  if(maxNormalForce<m_fMin)
    return false;
  m_fMax = maxNormalForce;
  Vector & ub = m_forceInequality.upperBound();
  ub(ub.size()-1) = m_fMax;
  return true;
}

void Contact6d::setForceReference(ConstRefVector & f_ref)
{
  m_fRef = f_ref;
  updateForceRegularizationTask();
}

void Contact6d::setReference(const SE3 & ref)
{
  TrajectorySample s(12, 6);
  SE3ToVector(ref, s.pos);
  m_motionTask.setReference(s);
}

const ConstraintBase & Contact6d::computeMotionTask(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    const Data & data)
{
  return m_motionTask.compute(t, q, v, data);
}

const ConstraintInequality & Contact6d::computeForceTask(const double,
                                                         ConstRefVector ,
                                                         ConstRefVector ,
                                                         const Data & )
{
  return m_forceInequality;
}

const Matrix & Contact6d::getForceGeneratorMatrix()
{
  return m_forceGenMat;
}

const ConstraintEquality & Contact6d::
computeForceRegularizationTask(const double ,
			       ConstRefVector ,
			       ConstRefVector ,
			       const Data & )
{
  return m_forceRegTask;
}

double Contact6d::getMinNormalForce() const { return m_fMin; }
double Contact6d::getMaxNormalForce() const { return m_fMax; }

const TaskMotion & Contact6d::getMotionTask() const { return m_motionTask; }

const ConstraintBase & Contact6d::getMotionConstraint() const { return m_motionTask.getConstraint(); }

const ConstraintInequality & Contact6d::getForceConstraint() const { return m_forceInequality; }

const ConstraintEquality & Contact6d::getForceRegularizationTask() const { return m_forceRegTask; }

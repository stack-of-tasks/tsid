//
// Copyright (c) 2017 CNRS
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

#include "tsid/solvers/solver-HQP-eiquadprog.hpp"
#include "tsid/math/utils.hpp"
#include "tsid/solvers/eiquadprog_2011.hpp"
#include "tsid/utils/stop-watch.hpp"

using namespace tsid::math;
using namespace tsid::solvers;
using namespace Eigen;

SolverHQuadProg::SolverHQuadProg(const std::string & name):
  SolverHQPBase(name),
  m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
{
  m_n = 0;
  m_neq = 0;
  m_nin = 0;
}

void SolverHQuadProg::sendMsg(const std::string & s)
{
  std::cout<<"[SolverHQuadProg."<<m_name<<"] "<<s<<std::endl;
}

void SolverHQuadProg::resize(unsigned int n, unsigned int neq, unsigned int nin)
{
  const bool resizeVar = n!=m_n;
  const bool resizeEq = (resizeVar || neq!=m_neq );
  const bool resizeIn = (resizeVar || nin!=m_nin );

  if(resizeEq)
  {
#ifndef NDEBUG
    sendMsg("Resizing equality constraints from "+toString(m_neq)+" to "+toString(neq));
#endif
    m_CE.resize(neq, n);
    m_ce0.resize(neq);
  }
  if(resizeIn)
  {
#ifndef NDEBUG
    sendMsg("Resizing inequality constraints from "+toString(m_nin)+" to "+toString(nin));
#endif
    m_CI.resize(2*nin, n);
    m_ci0.resize(2*nin);
  }
  if(resizeVar)
  {
#ifndef NDEBUG
    sendMsg("Resizing Hessian from "+toString(m_n)+" to "+toString(n));
#endif
    m_H.resize(n, n);
    m_g.resize(n);
    m_output.x.resize(n);
  }

  m_n = n;
  m_neq = neq;
  m_nin = nin;
}

const HQPOutput & SolverHQuadProg::solve(const HQPData & problemData)
{
  if(problemData.size()>2)
  {
    assert(false && "Solver not implemented for more than 2 hierarchical levels.");
  }

  // Compute the constraint matrix sizes
  unsigned int neq = 0, nin = 0;
  const ConstraintLevel & cl0 = problemData[0];
  if(cl0.size()>0)
  {
    const unsigned int n = cl0[0].second->cols();
    for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
    {
      const ConstraintBase* constr = it->second;
      assert(n==constr->cols());
      if(constr->isEquality())
        neq += constr->rows();
      else
        nin += constr->rows();
    }
    // If necessary, resize the constraint matrices
    resize(n, neq, nin);

    int i_eq=0, i_in=0;
    for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
    {
      const ConstraintBase* constr = it->second;
      if(constr->isEquality())
      {
        m_CE.middleRows(i_eq, constr->rows()) = constr->matrix();
        m_ce0.segment(i_eq, constr->rows())   = -constr->vector();
        i_eq += constr->rows();
      }
      else if(constr->isInequality())
      {
        m_CI.middleRows(i_in, constr->rows()) = constr->matrix();
        m_ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
        i_in += constr->rows();
        m_CI.middleRows(i_in, constr->rows()) = -constr->matrix();
        m_ci0.segment(i_in, constr->rows())   = constr->upperBound();
        i_in += constr->rows();
      }
      else if(constr->isBound())
      {
        m_CI.middleRows(i_in, constr->rows()).setIdentity();
        m_ci0.segment(i_in, constr->rows())   = -constr->lowerBound();
        i_in += constr->rows();
        m_CI.middleRows(i_in, constr->rows()) = -Matrix::Identity(m_n, m_n);
        m_ci0.segment(i_in, constr->rows())   = constr->upperBound();
        i_in += constr->rows();
      }
    }
  }
  else
    resize(m_n, neq, nin);

  if(problemData.size()>1)
  {
    const ConstraintLevel & cl1 = problemData[1];
    m_H.setZero();
    m_g.setZero();
    for(ConstraintLevel::const_iterator it=cl1.begin(); it!=cl1.end(); it++)
    {
      const double & w = it->first;
      const ConstraintBase* constr = it->second;
      if(!constr->isEquality())
        assert(false && "Inequalities in the cost function are not implemented yet");

      m_H += w*constr->matrix().transpose()*constr->matrix();
      m_g -= w*(constr->matrix().transpose()*constr->vector());
    }
    m_H.diagonal() += m_hessian_regularization*Vector::Ones(m_n);
  }

#ifdef ELIMINATE_EQUALITY_CONSTRAINTS

  // eliminate equality constraints
  const int r = m_neq;
  Matrix Z(m_n, m_n-m_neq);
  Matrix ZT(m_n, m_n);
  m_ZT_H_Z.resize(m_n-r, m_n-r);
  
  START_PROFILER("Eiquadprog eliminate equalities");
  if(m_neq>0)
  {
	START_PROFILER("Eiquadprog CE decomposition");
//    m_CE_dec.compute(m_CE, ComputeThinU | ComputeThinV);
	m_CE_dec.compute(m_CE);
	STOP_PROFILER("Eiquadprog CE decomposition");

	START_PROFILER("Eiquadprog get CE null-space basis");
// get nullspace basis from SVD	
//    const int r = m_CE_dec.nonzeroSingularValues();
//    const Matrix Z = m_CE_dec.matrixV().rightCols(m_n-r);

	// get null space basis from ColPivHouseholderQR
//	Matrix Z = m_CE_dec.householderQ();
//	Z = Z.rightCols(m_n-r);
	
	// get null space basis from COD
	// P^{-1} * y => colsPermutation() * y;
//	Z = m_CE_dec.matrixZ(); // * m_CE_dec.colsPermutation();
	ZT.setIdentity();
	//m_CE_dec.applyZAdjointOnTheLeftInPlace(ZT);
	typedef tsid::math::Index Index;
        const Index rank = m_CE_dec.rank();
        Vector temp(m_n);
        for (Index k = 0; k < rank; ++k)
        {
          if (k != rank - 1)
            ZT.row(k).swap(ZT.row(rank - 1));
          ZT.middleRows(rank - 1, m_n - rank + 1)
              .applyHouseholderOnTheLeft(
                m_CE_dec.matrixQTZ().row(k).tail(m_n - rank).adjoint(),
                m_CE_dec.zCoeffs()(k),
                &temp(0));
          if (k != rank - 1)
            ZT.row(k).swap(ZT.row(rank - 1));
        }
	STOP_PROFILER("Eiquadprog get CE null-space basis");
		
	// find a solution for the equalities
    Vector x0 = m_CE_dec.solve(m_ce0);
	x0 = -x0;

//    START_PROFILER("Eiquadprog project Hessian full");
//    m_ZT_H_Z.noalias() = Z.transpose()*m_H*Z; // this is too slow
//	STOP_PROFILER("Eiquadprog project Hessian full");
	
	START_PROFILER("Eiquadprog project Hessian incremental");
    const ConstraintLevel & cl1 = problemData[1];
    m_ZT_H_Z.setZero();
    //m_g.setZero();
	Matrix AZ;
    for(ConstraintLevel::const_iterator it=cl1.begin(); it!=cl1.end(); it++)
    {
      const double & w = it->first;
      const ConstraintBase* constr = it->second;
      if(!constr->isEquality())
        assert(false && "Inequalities in the cost function are not implemented yet");

	  AZ.noalias() = constr->matrix()*Z.rightCols(m_n-r);
      m_ZT_H_Z += w*AZ.transpose()*AZ;
      //m_g -= w*(constr->matrix().transpose()*constr->vector());
    }
    //m_ZT_H_Z.diagonal() += 1e-8*Vector::Ones(m_n);
    m_CI_Z.noalias() = m_CI*Z.rightCols(m_n-r);
    STOP_PROFILER("Eiquadprog project Hessian incremental");

  }
  STOP_PROFILER("Eiquadprog eliminate equalities");
#endif

//#ifndef NDEBUG
//  PRINT_MATRIX(m_H);
//  PRINT_VECTOR(m_g);
//  PRINT_MATRIX(m_CE);
//  PRINT_VECTOR(m_ce0);
//  PRINT_MATRIX(m_CI);
//  PRINT_VECTOR(m_ci0);
//#endif

  //  min 0.5 * x G x + g0 x
  //  s.t.
  //  CE^T x + ce0 = 0
  //  CI^T x + ci0 >= 0
  m_objValue = solve_quadprog(m_H, m_g, m_CE.transpose(), m_ce0,
                              m_CI.transpose(), m_ci0,
                              m_output.x, m_activeSet, m_activeSetSize);

  if(m_objValue==std::numeric_limits<double>::infinity())
    m_output.status = HQP_STATUS_INFEASIBLE;
  else
  {
    m_output.status = HQP_STATUS_OPTIMAL;
#ifndef NDEBUG
    const Vector & x = m_output.x;

    if(cl0.size()>0)
    {
      for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
      {
        const ConstraintBase* constr = it->second;
        if(constr->checkConstraint(x)==false)
        {
          if(constr->isEquality())
          {
            sendMsg("Equality "+constr->name()+" violated: "+
                    toString((constr->matrix()*x-constr->vector()).norm()));
          }
          else if(constr->isInequality())
          {
            sendMsg("Inequality "+constr->name()+" violated: "+
                    toString((constr->matrix()*x-constr->lowerBound()).minCoeff())+"\n"+
                    toString((constr->upperBound()-constr->matrix()*x).minCoeff()));
          }
          else if(constr->isBound())
          {
            sendMsg("Bound "+constr->name()+" violated: "+
                    toString((x-constr->lowerBound()).minCoeff())+"\n"+
                    toString((constr->upperBound()-x).minCoeff()));
          }
        }
      }
    }
#endif
  }

  return m_output;
}

double SolverHQuadProg::getObjectiveValue()
{
  return m_objValue;
}

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

#include <pininvdyn/solvers/solver-HQP-eiquadprog.hpp>
#include <pininvdyn/solvers/eiquadprog_2011.hpp>

using namespace pininvdyn::math;
using namespace pininvdyn::solvers;

Solver_HQP_eiquadprog::Solver_HQP_eiquadprog(const std::string & name):
  Solver_HQP_base(name)
{
  m_n = 0;
  m_neq = 0;
  m_nin = 0;
}

void Solver_HQP_eiquadprog::resize(unsigned int n, unsigned int neq, unsigned int nin)
{
  const bool resizeVar = n!=m_n;
  const bool resizeEq = (resizeVar || neq!=m_neq );
  const bool resizeIn = (resizeVar || nin!=m_nin );

  m_n = n;
  m_neq = neq;
  m_nin = nin;

  if(resizeEq)
  {
    m_CE.resize(m_neq, m_n);
    m_ce0.resize(m_neq);
  }
  if(resizeIn)
  {
    m_CI.resize(m_nin, m_n);
    m_ci0.resize(m_nin);
  }
  if(resizeVar)
  {
    m_H.resize(n, n);
    m_g.resize(n);
    m_output.x.resize(m_n);
  }
}

const HqpOutput & Solver_HQP_eiquadprog::solve(const HqpData & problemData)
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
        nin += 2*constr->rows();
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
    for(ConstraintLevel::const_iterator it=cl1.begin(); it!=cl1.end(); it++)
    {
      const double & w = it->first;
      const ConstraintBase* constr = it->second;
      if(!constr->isEquality())
        assert(false && "Inequalities in the cost function are not implemented yet");

      m_H += w*constr->matrix().transpose()*constr->matrix();
      m_g -= w*(constr->matrix().transpose()*constr->vector());
    }
  }

  //  min 0.5 * x G x + g0 x
  //  s.t.
  //  CE^T x + ce0 = 0
  //  CI^T x + ci0 >= 0
  m_objValue = solve_quadprog(m_H, m_g, m_CE.transpose(), m_ce0,
                              m_CI.transpose(), m_ci0,
                              m_output.x, m_activeSet, m_activeSetSize);

  if(m_objValue==std::numeric_limits<double>::infinity())
  {
    m_output.status = HQP_STATUS_INFEASIBLE;
  }
  return m_output;
}

double Solver_HQP_eiquadprog::getObjectiveValue()
{
  return m_objValue;
}

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

#include "tsid/solvers/solver-HQP-qpmad.hpp"
#include "tsid/math/utils.hpp"
#include "tsid/utils/stop-watch.hpp"

using namespace tsid::math;
using namespace tsid::solvers;
using namespace Eigen;

namespace tsid
{
  namespace solvers
  {

    SolverHQpmad::SolverHQpmad(const std::string & name):
      SolverHQPBase(name),
      m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
    {
      m_n = 0;
      m_nc = 0;
    }

    void SolverHQpmad::sendMsg(const std::string & s)
    {
      std::cout<<"[SolverHQpmad."<<m_name<<"] "<<s<<std::endl;
    }

    void SolverHQpmad::resize(unsigned int n, unsigned int neq, unsigned int nin)
    {
      unsigned int nc = neq+nin;

      const bool resizeVar = n!=m_n;
      const bool resizeEqIn = (resizeVar || nc!=m_nc);

      if(resizeEqIn)
      {
    #ifndef NDEBUG
        sendMsg("Resizing equality-inequality constraints from "+toString(m_nc)+" to "+toString(nc));
    #endif
        m_C.resize(nc, n);
        m_cl.resize(nc);
        m_cu.resize(nc);
      }
      if(resizeVar)
      {
    #ifndef NDEBUG
        sendMsg("Resizing Hessian from "+toString(m_n)+" to "+toString(n));
    #endif
        m_H.resize(n, n);
        m_g.resize(n);
        m_output.x.resize(n);

        if(m_has_bounds)
        {
          m_lb.resize(n);
          m_ub.resize(n);
        }
      }

      m_n = n;
      m_nc = nc;
    }

    const HQPOutput & SolverHQpmad::solve(const HQPData & problemData)
    {
      if(problemData.size()>2)
      {
        assert(false && "Solver not implemented for more than 2 hierarchical levels.");
      }

      // Compute the constraint matrix sizes
      m_has_bounds = false;
      unsigned int nin = 0, neq=0;
      const ConstraintLevel & cl0 = problemData[0];
      if(cl0.size()>0)
      {
        const unsigned int n = cl0[0].second->cols();
        for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
        {
          auto constr = it->second;
          assert(n==constr->cols());
          if(constr->isEquality())
            neq += constr->rows();
          else if(constr->isInequality())
            nin += constr->rows();
          else if(constr->isBound())
            m_has_bounds = true;
        }
        // If necessary, resize the constraint matrices
        resize(n, neq, nin);

        int i_eq_in=0;
        for(ConstraintLevel::const_iterator it=cl0.begin(); it!=cl0.end(); it++)
        {
          auto constr = it->second;
          if(constr->isEquality())
          {
            m_C.middleRows(i_eq_in, constr->rows()) = constr->matrix();
            m_cl.segment(i_eq_in, constr->rows())   = constr->vector();
            m_cu.segment(i_eq_in, constr->rows())   = constr->vector();
            i_eq_in += constr->rows();
          }
          else if(constr->isInequality())
          {
            m_C.middleRows(i_eq_in, constr->rows()) = constr->matrix();
            m_cl.segment(i_eq_in, constr->rows())   = constr->lowerBound();
            m_cu.segment(i_eq_in, constr->rows())   = constr->upperBound();
            i_eq_in += constr->rows();
          }
          else if(constr->isBound())
          {
            // not considering masks
            // considering a single bound constraint

            m_lb = constr->lowerBound();
            m_ub = constr->upperBound();
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
          auto constr = it->second;
          if(!constr->isEquality())
            assert(false && "Inequalities in the cost function are not implemented yet");

          m_H += w*constr->matrix().transpose()*constr->matrix();
          m_g -= w*(constr->matrix().transpose()*constr->vector());
        }
        m_H.diagonal() += m_hessian_regularization * Vector::Ones(m_n);
      }

      //  min 0.5 * x H x + g x
      //  s.t.
      //  cl <= C^T x <= cu
      //  lb <= x <= ub

      qpmad::Solver::ReturnStatus solve_status;

      qpmad::SolverParameters params; 
      // params.hessian_type_ = qpmad::SolverParameters::HESSIAN_LOWER_TRIANGULAR;
      // params.tolerance_ = 1e-12;
      // params.max_iter_ = -1;
      // params.return_inverted_cholesky_factor_ = false;

      if(m_has_bounds)
        solve_status = m_solver.solve(m_output.x, m_H, m_g, m_lb, m_ub, m_C, m_cl, m_cu, params);
      else
        solve_status = m_solver.solve(m_output.x, m_H, m_g, Eigen::VectorXd(), Eigen::VectorXd(), m_C, m_cl, m_cu, params);

      if(solve_status != qpmad::Solver::OK)
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
            auto constr = it->second;
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

    double SolverHQpmad::getObjectiveValue()
    {
      return 0;
    }

  } // namespace solvers
} // namespace tsid
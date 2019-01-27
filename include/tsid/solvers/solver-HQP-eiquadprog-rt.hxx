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

#ifndef __invdyn_solvers_hqp_eiquadprog_rt_hxx__
#define __invdyn_solvers_hqp_eiquadprog_rt_hxx__

#include "tsid/solvers/solver-HQP-eiquadprog-rt.hpp"
#include "tsid/solvers/eiquadprog-rt.hxx"
#include "tsid/utils/stop-watch.hpp"
#include "tsid/math/utils.hpp"


#define PROFILE_EIQUADPROG_PREPARATION "EiquadprogRT problem preparation"
#define PROFILE_EIQUADPROG_SOLUTION "EiquadprogRT problem solution"

namespace tsid
{
  namespace solvers
  {
    
    template<int nVars, int nEqCon, int nIneqCon>
    SolverHQuadProgRT<nVars, nEqCon, nIneqCon>::SolverHQuadProgRT(const std::string & name)
    : SolverHQPBase(name)
    , m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION)
    {
      m_n = nVars;
      m_neq = nEqCon;
      m_nin = nIneqCon;
      m_output.resize(nVars, nEqCon, 2*nIneqCon);
    }
    
    template<int nVars, int nEqCon, int nIneqCon>
    void SolverHQuadProgRT<nVars, nEqCon, nIneqCon>::sendMsg(const std::string & s)
    {
      std::cout<<"[SolverHQuadProgRT."<<m_name<<"] "<<s<<std::endl;
    }
    
    template<int nVars, int nEqCon, int nIneqCon>
    void SolverHQuadProgRT<nVars, nEqCon, nIneqCon>::resize(unsigned int n,
							    unsigned int neq,
							    unsigned int nin)
    {
      assert(n==nVars);
      assert(neq==nEqCon);
      assert(nin==nIneqCon);
      if ((n!=nVars) || (neq!=nEqCon) || (nin!=nIneqCon))
	std::cerr << "[SolverHQuadProgRT] (n!=nVars) || (neq!=nEqCon) || (nin!=nIneqCon)" << std::endl;
    }
    
    template<int nVars, int nEqCon, int nIneqCon>
    const HQPOutput & SolverHQuadProgRT<nVars, nEqCon, nIneqCon>::solve(const HQPData & problemData)
    {
      using namespace tsid::math;
      
//#ifndef EIGEN_RUNTIME_NO_MALLOC
      //  Eigen::internal::set_is_malloc_allowed(false);
//#endif
      
      START_PROFILER_EIQUADPROG_RT(PROFILE_EIQUADPROG_PREPARATION);
      
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
          
          EIGEN_MALLOC_ALLOWED
          m_H.noalias() += w*constr->matrix().transpose()*constr->matrix();
          EIGEN_MALLOC_NOT_ALLOWED
          m_g.noalias() -= w*(constr->matrix().transpose()*constr->vector());
        }
        m_H.diagonal().noalias() += m_hessian_regularization*Vector::Ones(m_n);
      }
      
      STOP_PROFILER_EIQUADPROG_RT(PROFILE_EIQUADPROG_PREPARATION);
      
      //  // eliminate equality constraints
      //  if(m_neq>0)
      //  {
      //    m_CE_lu.compute(m_CE);
      //    sendMsg("The rank of CD is "+toString(m_CE_lu.rank());
      //    const MatrixXd & Z = m_CE_lu.kernel();
      
      //  }
      
      START_PROFILER_EIQUADPROG_RT(PROFILE_EIQUADPROG_SOLUTION);
      
      //  min 0.5 * x G x + g0 x
      //  s.t.
      //  CE x + ce0 = 0
      //  CI x + ci0 >= 0
      typename RtVectorX<nVars>::d sol(m_n);
      EIGEN_MALLOC_ALLOWED
      RtEiquadprog_status status = m_solver.solve_quadprog(m_H, m_g,
                                                           m_CE, m_ce0,
                                                           m_CI, m_ci0,
                                                           sol);
      STOP_PROFILER_EIQUADPROG_RT(PROFILE_EIQUADPROG_SOLUTION);
      
      m_output.x = sol;
      
//#ifndef EIGEN_RUNTIME_NO_MALLOC
      //  Eigen::internal::set_is_malloc_allowed(true);
//#endif
      
      if(status==RT_EIQUADPROG_OPTIMAL)
      {
        m_output.status = HQP_STATUS_OPTIMAL;
        m_output.lambda = m_solver.getLagrangeMultipliers();
        //    m_output.activeSet = m_solver.getActiveSet().template tail< 2*nIneqCon >().head(m_solver.getActiveSetSize());
        m_output.activeSet = m_solver.getActiveSet().segment(m_neq, m_solver.getActiveSetSize()-m_neq);
        m_output.iterations = m_solver.getIteratios();
        
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
      else if(status==RT_EIQUADPROG_UNBOUNDED)
        m_output.status = HQP_STATUS_INFEASIBLE;
      else if(status==RT_EIQUADPROG_MAX_ITER_REACHED)
        m_output.status = HQP_STATUS_MAX_ITER_REACHED;
      else if(status==RT_EIQUADPROG_REDUNDANT_EQUALITIES)
        m_output.status = HQP_STATUS_ERROR;
      
      return m_output;
    }
    
    template<int nVars, int nEqCon, int nIneqCon>
    double SolverHQuadProgRT<nVars, nEqCon, nIneqCon>::getObjectiveValue()
    {
      return m_solver.getObjValue();
    }
    
    template<int nVars, int nEqCon, int nIneqCon>
    bool SolverHQuadProgRT<nVars, nEqCon, nIneqCon>::setMaximumIterations(unsigned int maxIter)
    {
      SolverHQPBase::setMaximumIterations(maxIter);
      return m_solver.setMaxIter(maxIter);
    }
  
  } // namespace solvers
} // namespace tsid

#endif // ifndef __invdyn_solvers_hqp_eiquadprog_rt_hxx__

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

#ifndef __invdyn_solvers_hqp_base_hpp__
#define __invdyn_solvers_hqp_base_hpp__

#include <pininvdyn/math/utils.hpp>
#include <pininvdyn/math/constraint-base.hpp>
#include <pininvdyn/config.hh>

#include <vector>
#include <utility>

namespace pininvdyn
{
  namespace solvers
  {

    /**
    * Available HQP solvers.
    */
    enum PININVDYN_DLLAPI SolverHQP
    {
      SOLVER_HQP_EIQUADPROG = 0
#ifdef QPOASES_FOUND
      ,SOLVER_HQP_OASES = 1
#endif
    };


    /**
    * Possible states of an HQP solver.
    */
    enum PININVDYN_DLLAPI HQP_status
    {
      HQP_STATUS_UNKNOWN=-1,
      HQP_STATUS_OPTIMAL=0,
      HQP_STATUS_INFEASIBLE=1,
      HQP_STATUS_UNBOUNDED=2,
      HQP_STATUS_MAX_ITER_REACHED=3,
      HQP_STATUS_ERROR=4
    };

    typedef std::vector< std::pair<double, pininvdyn::math::ConstraintBase*> > ConstraintLevel;
    typedef std::vector< std::pair<double, const pininvdyn::math::ConstraintBase*> > ConstConstraintLevel;
    typedef std::vector<ConstraintLevel> HqpData;
    typedef std::vector<ConstConstraintLevel> ConstHqpData;

    std::string hqpDataToString(const HqpData & data, bool printMatrices=false);

    class HqpOutput
    {
    public:
      HQP_status status;
      pininvdyn::math::Vector x, lambda;
    };

    /**
     * @brief Abstract interface for a Quadratic Program (HQP) solver.
     */
    class PININVDYN_DLLAPI Solver_HQP_base
    {
    public:
      typedef pininvdyn::math::RefVector RefVector;
      typedef pininvdyn::math::ConstRefVector ConstRefVector;
      typedef pininvdyn::math::ConstRefMatrix ConstRefMatrix;

      Solver_HQP_base(const std::string & name);

      /**
       * @brief Create a new HQP solver of the specified type.
       * @param solverType Type of HQP solver.
       * @return A pointer to the new solver.
       */
      static Solver_HQP_base* getNewSolver(SolverHQP solverType, const std::string & name);

      virtual const std::string & name(){ return m_name; }

      virtual void resize(unsigned int n, unsigned int neq, unsigned int nin) = 0;

      /** Solve the specified Hierarchical Quadratic Program.
       */
      virtual const HqpOutput & solve(const HqpData & problemData) = 0;

      /** Get the objective value of the last solved problem. */
      virtual double getObjectiveValue() = 0;

      /** Return true if the solver is allowed to warm start, false otherwise. */
      virtual bool getUseWarmStart(){ return m_useWarmStart; }
      /** Specify whether the solver is allowed to use warm-start techniques. */
      virtual void setUseWarmStart(bool useWarmStart){ m_useWarmStart = useWarmStart; }

      /** Get the current maximum number of iterations performed by the solver. */
      virtual unsigned int getMaximumIterations(){ return m_maxIter; }
      /** Set the current maximum number of iterations performed by the solver. */
      virtual bool setMaximumIterations(unsigned int maxIter);


      /** Get the maximum time allowed to solve a problem. */
      virtual double getMaximumTime(){ return m_maxTime; }
      /** Set the maximum time allowed to solve a problem. */
      virtual bool setMaximumTime(double seconds);

    protected:
      std::string           m_name;
      bool                  m_useWarmStart;   // true if the solver is allowed to warm start
      int                   m_maxIter;        // max number of iterations
      double                m_maxTime;        // max time to solve the HQP [s]
      HqpOutput             m_output;
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_base_hpp__

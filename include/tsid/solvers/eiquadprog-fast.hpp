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

#ifndef EIQUADPROGFAST_HH_
#define EIQUADPROGFAST_HH_

#include <Eigen/Dense>
#include <tsid/math/fwd.hpp>

#define OPTIMIZE_STEP_1_2   // compute s(x) = ci^T * x + ci0
#define OPTIMIZE_COMPUTE_D
#define OPTIMIZE_UPDATE_Z
#define OPTIMIZE_HESSIAN_INVERSE
#define OPTIMIZE_UNCONSTR_MINIM

//#define USE_WARM_START
//#define PROFILE_EIQUADPROG

//#define DEBUG_STREAM(msg) std::cout<<msg;
#define DEBUG_STREAM(msg)

#ifdef PROFILE_EIQUADPROG
#define START_PROFILER_EIQUADPROG_FAST(x) START_PROFILER(x)
#define STOP_PROFILER_EIQUADPROG_FAST(x)  STOP_PROFILER(x)
#else
#define START_PROFILER_EIQUADPROG_FAST(x)
#define STOP_PROFILER_EIQUADPROG_FAST(x)
#endif

#define EIQUADPROG_FAST_CHOWLESKY_DECOMPOSITION "EIQUADPROG_FAST Chowlesky dec"
#define EIQUADPROG_FAST_CHOWLESKY_INVERSE      "EIQUADPROG_FAST Chowlesky inv"
#define EIQUADPROG_FAST_ADD_EQ_CONSTR          "EIQUADPROG_FAST ADD_EQ_CONSTR"
#define EIQUADPROG_FAST_ADD_EQ_CONSTR_1        "EIQUADPROG_FAST ADD_EQ_CONSTR_1"
#define EIQUADPROG_FAST_ADD_EQ_CONSTR_2        "EIQUADPROG_FAST ADD_EQ_CONSTR_2"
#define EIQUADPROG_FAST_STEP_1                 "EIQUADPROG_FAST STEP_1"
#define EIQUADPROG_FAST_STEP_1_1               "EIQUADPROG_FAST STEP_1_1"
#define EIQUADPROG_FAST_STEP_1_2               "EIQUADPROG_FAST STEP_1_2"
#define EIQUADPROG_FAST_STEP_1_UNCONSTR_MINIM  "EIQUADPROG_FAST STEP_1_UNCONSTR_MINIM"
#define EIQUADPROG_FAST_STEP_2                 "EIQUADPROG_FAST STEP_2"
#define EIQUADPROG_FAST_STEP_2A                "EIQUADPROG_FAST STEP_2A"
#define EIQUADPROG_FAST_STEP_2B                "EIQUADPROG_FAST STEP_2B"
#define EIQUADPROG_FAST_STEP_2C                "EIQUADPROG_FAST STEP_2C"

#define DEFAULT_MAX_ITER 1000

namespace tsid
{

  namespace solvers
  {

    /**
    * Possible states of the solver.
    */
    enum EiquadprogFast_status
    {
      EIQUADPROG_FAST_OPTIMAL=0,
      EIQUADPROG_FAST_INFEASIBLE=1,
      EIQUADPROG_FAST_UNBOUNDED=2,
      EIQUADPROG_FAST_MAX_ITER_REACHED=3,
      EIQUADPROG_FAST_REDUNDANT_EQUALITIES=4
    };

    class EiquadprogFast
    {
      typedef Eigen::MatrixXd MatrixXd;
      typedef Eigen::VectorXd VectorXd;
      typedef Eigen::VectorXi VectorXi;

    public:
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      EiquadprogFast();
      virtual ~EiquadprogFast();

      void reset(math::Index dim_qp, math::Index num_eq, math::Index num_ineq);

      int getMaxIter() const { return m_maxIter; }

      bool setMaxIter(int maxIter)
      {
        if(maxIter<0)
          return false;
        m_maxIter = maxIter;
        return true;
      }

      /**
       * @return The size of the active set, namely the number of
       * active constraints (including the equalities).
       */
      math::Index getActiveSetSize() const { return q; }

      /**
       * @return The number of active-set iteratios.
       */
      int getIteratios() const { return iter; }

      /**
       * @return The value of the objective function.
       */
      double getObjValue() const { return f_value; }

      /**
       * @return The Lagrange multipliers
       */
      const VectorXd & getLagrangeMultipliers() const { return u; }

      /**
       * Return the active set, namely the indeces of active constraints.
       * The first nEqCon indexes are for the equalities and are negative.
       * The last nIneqCon indexes are for the inequalities and start from 0.
       * Only the first q elements of the return vector are valid, where q
       * is the size of the active set.
       * @return The set of indexes of the active constraints.
       */
      const VectorXi & getActiveSet() const { return A; }

      /**
       * solves the problem
       * min. x' Hess x + 2 g0' x
       * s.t. CE x + ce0 = 0
       *      CI x + ci0 >= 0
       */
       EiquadprogFast_status solve_quadprog(const MatrixXd & Hess,
                                            const VectorXd & g0,
                                            const MatrixXd & CE,
                                            const VectorXd & ce0,
                                            const MatrixXd & CI,
                                            const VectorXd & ci0,
                                            VectorXd & x);

      MatrixXd m_J; // J * J' = Hessian <nVars,nVars>::d
      bool is_inverse_provided_;

    private:
      math::Index m_nVars;
      math::Index m_nEqCon;
      math::Index m_nIneqCon;

      int m_maxIter;  /// max number of active-set iterations
      double f_value; /// current value of cost function

      Eigen::LLT<MatrixXd,Eigen::Lower> chol_; // <nVars,nVars>::d

      /// from QR of L' N, where L is Cholewsky factor of Hessian, and N is the matrix of active constraints
      MatrixXd R; // <nVars,nVars>::d

      /// CI*x+ci0
      VectorXd s; // <nIneqCon>::d

      /// infeasibility multipliers, i.e. negative step direction in dual space
      VectorXd r; // <nIneqCon+nEqCon>::d

      /// Lagrange multipliers
      VectorXd u; // <nIneqCon+nEqCon>::d

      /// step direction in primal space
      VectorXd z; // <nVars>::d

      /// J' np
      VectorXd d; //<nVars>::d

      /// current constraint normal
      VectorXd np; //<nVars>::d

      /// active set (indeces of active constraints)
      /// the first nEqCon indeces are for the equalities and are negative
      /// the last nIneqCon indeces are for the inequalities are start from 0
      VectorXi A; // <nIneqCon+nEqCon>

      /// initialized as K \ A
      /// iai(i)=-1 iff inequality constraint i is in the active set
      /// iai(i)=i otherwise
      VectorXi iai; // <nIneqCon>::i

      /// initialized as [1, ..., 1, .]
      /// if iaexcl(i)!=1 inequality constraint i cannot be added to the active set
      /// if adding ineq constraint i fails => iaexcl(i)=0
      /// iaexcl(i)=0 iff ineq constraint i is linearly dependent to other active constraints
      /// iaexcl(i)=1 otherwise
      VectorXi iaexcl; //<nIneqCon>::i

      VectorXd x_old;           // old value of x <nVars>::d
      VectorXd u_old; // old value of u <nIneqCon+nEqCon>::d
      VectorXi A_old; // old value of A <nIneqCon+nEqCon>::i
	  
#ifdef OPTIMIZE_ADD_CONSTRAINT
	  VectorXd T1; /// tmp variable used in add_constraint
#endif

      /// size of the active set A (containing the indices of the active constraints)
      math::Index q;

      /// number of active-set iterations
      int iter;

      inline void compute_d(VectorXd & d,
                            const MatrixXd & J,
                            const VectorXd & np)
      {
#ifdef OPTIMIZE_COMPUTE_D
        d.noalias() = J.adjoint() * np;
#else
        d = J.adjoint() * np;
#endif
      }

      inline void update_z(VectorXd & z,
                           const MatrixXd & J,
                           const VectorXd & d,
                           math::Index iq)
      {
#ifdef OPTIMIZE_UPDATE_Z
        z.noalias() = J.rightCols(z.size()-iq) * d.tail(z.size()-iq);
#else
        z = J.rightCols(J.cols()-iq) * d.tail(J.cols()-iq);
#endif
      }

      inline void update_r(const MatrixXd & R,
                           VectorXd & r,
                           const VectorXd & d,
                           math::Index iq)
      {
        r.head(iq)= d.head(iq);
        R.topLeftCorner(iq,iq).triangularView<Eigen::Upper>().solveInPlace(r.head(iq));
      }

      inline bool add_constraint(MatrixXd & R,
                                 MatrixXd & J,
                                 VectorXd & d,
                                 math::Index& iq, double& R_norm);

      inline void delete_constraint(MatrixXd & R,
                                    MatrixXd & J,
                                    VectorXi & A,
                                    VectorXd & u,
                                    math::Index nEqCon, math::Index& iq,
				    math::Index l);
    };

  } /* namespace solvers */
} /* namespace tsid */

#endif /* EIQUADPROGFAST_HH_ */
